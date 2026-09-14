#!/usr/bin/env python3
"""Launch-owned exploration session: ready -> start -> finish/stop -> save."""
import os
import math
import signal
import subprocess
import threading
import time
import json
import ctypes
from pathlib import Path

import rospy
from actionlib_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse, SetBool
from go2_exploration_safety.fault_recovery import RECOVERABLE_FAULTS, RecoveryDwell, sdk_recovery_problem

LIBC=ctypes.CDLL(None)


def parent_death_signal():
    # The goal producer cannot outlive a killed supervisor.
    LIBC.prctl(1, signal.SIGINT)
    if os.getppid()==1:
        os.kill(os.getpid(),signal.SIGINT)


class Supervisor:
    def __init__(self):
        self.lock = threading.RLock()
        self.auto_start = rospy.get_param('~auto_start', True)
        self.use_real_sdk = rospy.get_param('~use_real_sdk', False)
        self.auto_reenable = self.auto_start and rospy.get_param('~auto_reenable', True)
        self.recovery_dwell = RecoveryDwell(rospy.get_param('~recovery_stable_sec', 10.0))
        self.recovery_attempts = []
        self.recovery_block = ''
        self.gate_recovery_ready = False
        self.health_reason = 'waiting_for_safety'
        self.sdk_values = {}
        self.sdk_stamp = 0.0
        self.post_enable_settle = max(0.,float(rospy.get_param('~post_enable_settle_sec',5.0 if self.use_real_sdk else 0.0)))
        self.post_enable_stable = max(0.,float(rospy.get_param('~post_enable_stable_sec',3.0 if self.use_real_sdk else 0.0)))
        self.post_enable_timeout = max(self.post_enable_settle+self.post_enable_stable+2.,
                                      float(rospy.get_param('~post_enable_timeout_sec',20.0)))
        self.session_dir = rospy.get_param('~session_dir', '')
        self.stop_requested = False
        self.startup_failed = False
        self.bridge_enabled = False
        self.bridge_ever_enabled = False
        self.bridge_enable_attempted = False
        self.safety_healthy = False
        self.last_unhealthy_time = float('-inf')
        self.completion_hold = max(1.0, float(rospy.get_param('~completion_hold_time', 30.0)))
        self.completion_maps = max(2, int(rospy.get_param('~completion_map_updates', 2)))
        self.map_name = rospy.get_param('~map_name', 'current_exploration')
        self.map_timeout = float(rospy.get_param('~map_timeout', 2.0))
        self.map_status_topic = rospy.get_param('~map_status_topic', '/exploration/map_status')
        self.armed = False
        self.latched = True
        self.safety_reason = 'waiting_for_safety'
        self.available = self.frontier_valid = self.active = False
        self.frontier_since = self.empty_since = None
        self.empty_generation = 0
        self.idle_since = time.monotonic()
        self.safety_stamp = self.frontier_stamp = self.status_stamp = self.map_stamp = 0.0
        self.map_generation = 0
        self.last_map_message = None
        self.refresh_ok = False
        self.last_map_error = ''
        self.arm_requested = self.ever_armed = False
        self.arm_request_time = self.next_arm_try = 0.0
        self.restarts = 0
        self.child = None
        self.last_state = None
        self.result = 'STOPPED'
        self.paused = False
        self.resume_ready_since = None
        self.obstacle_hold = False
        self.obstacle_event = 0
        self.handled_obstacle_event = 0
        self.selected_goal = None
        self.recovery_blacklist = []
        self.state_publisher = rospy.Publisher('/exploration/state', String, queue_size=1, latch=True)
        self.recovery_publisher = rospy.Publisher('/exploration/recovery_status', String, queue_size=1, latch=True)
        self.recovery_publisher.publish(String(data=json.dumps(dict(
            enabled=bool(self.auto_reenable),reason='no_fault',stable_seconds=0,
            required_seconds=self.recovery_dwell.seconds,attempts_in_five_minutes=0))))
        self.auto_reset_service = rospy.ServiceProxy('/go2_exploration_safety/auto_reset', Trigger)
        self.arm_service = rospy.ServiceProxy('/go2_exploration_safety/arm', Trigger)
        self.stop_service = rospy.ServiceProxy('/go2_exploration_safety/stop', Trigger)
        self.pause_service = rospy.ServiceProxy('/go2_exploration_safety/pause', Trigger)
        self.resume_service = rospy.ServiceProxy('/go2_exploration_safety/resume', Trigger)
        self.save_service = rospy.ServiceProxy('/go2_map_builder/save_map', Trigger)
        bridge = '/go2_sdk_bridge_real' if self.use_real_sdk else '/go2_sdk_bridge_mock'
        self.enable_service = rospy.ServiceProxy(bridge+'/enable', SetBool)
        self.stop_api = rospy.Service('/exploration/stop', Trigger, self.request_stop)
        self.subs = [
            rospy.Subscriber('/go2_exploration_safety/status', DiagnosticArray, self.safety),
            rospy.Subscriber('/exploration/frontier_status', String, self.frontier),
            rospy.Subscriber('/move_base/status', GoalStatusArray, self.status),
            rospy.Subscriber(self.map_status_topic, String, self.map_status),
            rospy.Subscriber('/go2/control/enabled', Bool, self.bridge_status),
            rospy.Subscriber('/go2/control/state', String, self.control_state),
            rospy.Subscriber('/go2/diagnostics', DiagnosticArray, self.sdk_diagnostics),
        ]

    def control_state(self, message):
        if message.data in ('manual_ready', 'manual_override'):
            with self.lock:
                self.recovery_block = 'manual_control_takeover'
                self.recovery_dwell.reset()

    def sdk_diagnostics(self, message):
        with self.lock:
            for status in message.status:
                if status.name != 'GO2 SDK bridge':
                    continue
                self.sdk_values = {v.key: v.value for v in status.values}
                self.sdk_values['_diagnostic_level'] = str(status.level)
                self.sdk_stamp = time.monotonic()
                problem = sdk_recovery_problem(self.sdk_values)
                if problem:
                    self.recovery_dwell.reset()
                if self.bridge_ever_enabled and (
                        self.sdk_values.get('no_step_response') == 'true'
                        or self.sdk_values.get('manual_resume_pending') == 'true'
                        or self.sdk_values.get('mode_override_api', '0') != '0'):
                    self.recovery_block = problem or 'manual_or_motion_fault'

    def bridge_status(self, message):
        with self.lock:
            self.bridge_enabled = bool(message.data)
            self.bridge_ever_enabled |= self.bridge_enabled

    def request_stop(self, _request):
        self.stop_requested = True
        self.recovery_block = 'operator_stop'
        try:
            self.stop_service()
        except rospy.ServiceException:
            pass
        self.disable_bridge()
        return TriggerResponse(success=True, message='Motion disabled; cancelling and saving this session')

    def disable_bridge(self):
        try:
            self.enable_service(False)
        except rospy.ServiceException as error:
            rospy.logwarn('SDK disable unavailable: %s', error)

    def map_status(self, message):
        with self.lock:
            if message.data.startswith('ready:'):
                self.refresh_ok = True
                self.last_map_error = ''
                if message.data != self.last_map_message:
                    self.map_generation += 1
                    self.map_stamp = time.monotonic()
                    self.last_map_message = message.data
            elif message.data.startswith('refresh failed:'):
                self.refresh_ok = False
                self.last_map_error = message.data
                # A skipped observation does not invalidate the last good map.
                # It must not count toward completion or refresh its freshness.
                self.empty_since = None

    def publish_state(self, state):
        if state != self.last_state:
            self.state_publisher.publish(String(data=state))
            rospy.loginfo('Exploration state: %s', state)
            self.last_state = state

    def safety(self, message):
        with self.lock:
            for status in message.status:
                if status.name == 'go2_exploration_safety':
                    values = {v.key: v.value for v in status.values}
                    self.latched = values.get('latched_stop') != 'False'
                    self.armed = values.get('armed') == 'True' and not self.latched
                    self.ever_armed |= self.armed
                    self.safety_reason = (values.get('latched_reason') if self.latched else None) or values.get('reason', status.message)
                    self.safety_stamp = time.monotonic()
                    self.safety_healthy = values.get('health_ok') == 'True'
                    self.health_reason = values.get('health_reason', 'unknown')
                    self.gate_recovery_ready = values.get('auto_recovery_ready') == 'True'
                    if values.get('auto_recovery_inhibited') == 'True':
                        self.recovery_block = 'gate_recovery_inhibited: '+self.safety_reason
                    if not self.safety_healthy or not self.gate_recovery_ready:
                        self.recovery_dwell.reset()
                    if not self.safety_healthy:self.last_unhealthy_time=time.monotonic()
                    self.obstacle_hold = values.get('obstacle_hold') == 'True'
                    self.obstacle_event = int(values.get('obstacle_event', '0'))
                    if self.obstacle_event != self.handled_obstacle_event:
                        # Capture the gate's interrupted target, not a possibly
                        # newer latched RViz marker published after cancellation.
                        self.selected_goal = None
                        try:
                            x, y = float(values['obstacle_goal_x']), float(values['obstacle_goal_y'])
                            if values.get('obstacle_goal_frame') and math.isfinite(x) and math.isfinite(y):
                                self.selected_goal = PoseStamped()
                                self.selected_goal.header.frame_id = values['obstacle_goal_frame']
                                self.selected_goal.pose.position.x = x
                                self.selected_goal.pose.position.y = y
                        except (KeyError, ValueError):
                            pass

    def frontier(self, message):
        # One atomic status distinguishes a valid empty map from missing TF/data.
        with self.lock:
            now = time.monotonic()
            self.frontier_valid = message.data in ('AVAILABLE', 'EMPTY')
            available = message.data == 'AVAILABLE'
            if available and (not self.available or now-self.frontier_stamp >= 2.5):
                self.frontier_since = now
            if not available:
                self.frontier_since = None
            self.available = available
            self.frontier_stamp = now

    def status(self, message):
        with self.lock:
            now = time.monotonic()
            active = any(s.status in (0, 1, 6, 7) for s in message.status_list)
            if self.active or active:
                self.idle_since = now
            self.active = active
            self.status_stamp = now

    def data_ready(self, now):
        return self.data_problem(now) is None

    def data_problem(self, now):
        if now-self.safety_stamp >= 1.0:
            return 'safety_status_stale'
        if now-self.status_stamp >= 1.0:
            return 'move_base_status_stale'
        if self.map_stamp <= 0.0:
            return 'no_successful_observation_map'
        if now-self.map_stamp >= self.map_timeout:
            return 'observation_map_stale: ' + self.last_map_error
        if not self.frontier_valid:
            return 'frontier_invalid'
        if now-self.frontier_stamp >= 2.5:
            return 'frontier_status_stale'
        return None

    def can_run(self, now):
        return (self.armed and self.bridge_enabled and not self.latched and not self.obstacle_hold
                and self.data_ready(now))

    def stable_frontier(self, now):
        return (self.available and self.frontier_since is not None
                and now-self.frontier_since >= 2.0)

    def can_start(self, now):
        return self.can_run(now) and self.stable_frontier(now)

    def should_arm(self, now):
        return (self.auto_start and not self.arm_requested and not self.ever_armed
                and not self.bridge_enable_attempted and self.safety_healthy
                and not self.latched and not self.armed and not self.active
                and now >= self.next_arm_try and self.data_ready(now)
                and self.refresh_ok and self.stable_frontier(now))

    def completed(self, now):
        enabled = self.auto_start or self.ever_armed
        if (not enabled or self.paused or self.obstacle_hold
                or self.obstacle_event != self.handled_obstacle_event
                or not self.data_ready(now) or not self.refresh_ok
                or self.latched or self.active or self.available):
            self.empty_since = None
            return False
        if self.empty_since is None:
            self.empty_since = now
            self.empty_generation = self.map_generation
        # Wait for a frontier observation after the newest successful map load.
        if self.frontier_stamp < self.map_stamp:
            return False
        return (now-self.empty_since >= self.completion_hold
                and self.map_generation-self.empty_generation >= self.completion_maps)

    def can_restart(self, now):
        return (self.restarts < 2 and self.can_start(now) and not self.active
                and now-self.frontier_since >= 8.0 and now-self.idle_since >= 15.0)

    def start(self):
        now = rospy.Time.now().to_sec()
        self.recovery_blacklist = [g for g in self.recovery_blacklist if g['until'] > now]
        rospy.set_param('/explore/recovery_blacklist', self.recovery_blacklist)
        self.child = subprocess.Popen(
            ['rosrun', 'go2_explore_lite', 'go2_explore',
             '__name:=explore', '_preview_only:=false'], start_new_session=True,
            preexec_fn=parent_death_signal)

    def stop_child(self):
        child = self.child
        if child is not None and child.poll() is None:
            for sig, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                try:
                    os.killpg(child.pid, sig)
                    child.wait(timeout=timeout)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.child = None

    def request_arm(self, now, recovery=False):
        # Initial enable and a qualified recovery both use the preserved SDK
        # checks and posture preparation. A refusal is never retried in a loop.
        self.bridge_enable_attempted = True
        try:
            if self.stop_requested or (recovery and self.recovery_block):
                raise RuntimeError('enable cancelled by operator or nonrecoverable fault')
            enabled = self.enable_service(True)
            if not enabled.success:
                raise RuntimeError(enabled.message)
            deadline=time.monotonic()+2.0
            while not self.bridge_enabled and time.monotonic()<deadline and not self.stop_requested and not rospy.is_shutdown():
                time.sleep(.02)
            if not self.bridge_enabled or self.stop_requested or rospy.is_shutdown():
                raise RuntimeError('SDK enable not acknowledged or session stopped')
            self.wait_for_post_enable_stability()
            response = self.arm_service()
            if not response.success:
                raise RuntimeError(response.message)
        except rospy.ServiceException as error:
            self.startup_failed = True
            self.safety_reason = 'startup_enable_failed: '+str(error)
            self.disable_bridge()
            return False
        except RuntimeError as error:
            self.startup_failed = True
            self.safety_reason = 'startup_enable_failed: '+str(error)
            self.disable_bridge()
            return False
        if response.success:
            self.arm_requested = True
            self.arm_request_time = time.monotonic()
            rospy.loginfo('Launch startup accepted by safety gate; beginning exploration')
            return True
        else:
            rospy.logwarn_throttle(5.0, 'Waiting for safety readiness: %s', response.message)

    def wait_for_post_enable_stability(self):
        # SDK acknowledgement may precede the end of its posture preparation.
        # Keep the velocity gate UNARMED throughout this bounded settling phase.
        # Once armed, all existing hard-fault latches remain unchanged.
        if self.post_enable_settle==0 and self.post_enable_stable==0:return
        start=time.monotonic();stable_since=None
        self.publish_state('WAITING_FOR_POSTURE_STABILITY')
        while True:
            now=time.monotonic()
            with self.lock:
                if (self.stop_requested or rospy.is_shutdown() or not self.bridge_enabled
                        or self.latched or self.armed or self.recovery_block):
                    raise RuntimeError('post-enable preparation interrupted; no automatic retry')
                healthy=(self.safety_healthy and self.data_ready(now) and self.refresh_ok and not self.active)
            if now-start>=self.post_enable_timeout:
                raise RuntimeError('post-enable perception did not stabilize before timeout')
            if stable_since is not None and self.last_unhealthy_time>=stable_since:stable_since=None
            if now-start<self.post_enable_settle or not healthy:stable_since=None
            elif stable_since is None:stable_since=now
            if stable_since is not None and now-stable_since>=self.post_enable_stable:return
            time.sleep(.05)

    def hold_session(self, fault, obstacle=False):
        # Upgrading an existing obstacle/data pause to a fault must also disable.
        if fault and self.bridge_enabled and self.paused:
            self.disable_bridge()
        if not self.paused:
            if fault:
                self.disable_bridge()
            if not fault and not obstacle:
                rospy.logwarn('Exploration data pause: %s', self.data_problem(time.monotonic()))
            if not self.latched and not obstacle:
                try:
                    response = self.pause_service()
                    if not response.success:
                        self.stop_service()
                except rospy.ServiceException:
                    try:
                        self.stop_service()
                    except rospy.ServiceException:
                        rospy.logerr('Safety pause unavailable; cancelling goal producer')
            self.stop_child()
            self.paused = True
        self.empty_since = None
        self.resume_ready_since = None
        if not (fault and self.last_state == 'WAITING_FOR_FAULT_RECOVERY'):
            self.publish_state('FAULT_STOPPED' if fault else
                               'WAITING_FOR_OBSTACLE' if obstacle else 'PAUSED_WAITING_FOR_DATA')

    def recovery_problem(self, now):
        if not self.auto_reenable:
            return 'automatic_reenable_disabled'
        if self.stop_requested or self.recovery_block:
            return self.recovery_block or 'operator_stop'
        if self.startup_failed or not self.ever_armed:
            return 'no_successful_prior_arm_or_enable_refused'
        if not self.latched or self.safety_reason not in RECOVERABLE_FAULTS:
            return 'nonrecoverable_fault: '+self.safety_reason
        if self.bridge_enabled or self.armed or self.active or self.child is not None:
            return 'waiting_for_disabled_control_and_cancelled_goal'
        problem = self.data_problem(now)
        if problem:
            return problem
        if not self.refresh_ok or not self.safety_healthy:
            return self.health_reason
        if not self.gate_recovery_ready:
            return 'waiting_for_standstill_and_clear_stop_region'
        if self.use_real_sdk:
            if now-self.sdk_stamp > 1.5:
                return 'sdk_diagnostics_stale'
            problem = sdk_recovery_problem(self.sdk_values)
            if problem:
                return problem
        self.recovery_attempts = [t for t in self.recovery_attempts if now-t < 300.0]
        if len(self.recovery_attempts) >= 3:
            return 'recovery_rate_limit: three attempts in five minutes'
        if self.recovery_attempts and now-self.recovery_attempts[-1] < 30.0:
            return 'recovery_cooldown'
        return None

    def recover_if_ready(self, now):
        with self.lock:
            problem = self.recovery_problem(now)
            ready = self.recovery_dwell.update(now, self.safety_reason, problem is None, self.map_generation)
            elapsed = 0.0 if self.recovery_dwell.since is None else now-self.recovery_dwell.since
            waiting = (self.auto_reenable and self.ever_armed and not self.startup_failed
                       and not self.recovery_block and self.safety_reason in RECOVERABLE_FAULTS)
        self.recovery_publisher.publish(String(data=json.dumps({
            'enabled': bool(self.auto_reenable), 'reason': problem or 'confirming_stable_recovery',
            'stable_seconds': round(elapsed, 1), 'required_seconds': self.recovery_dwell.seconds,
            'attempts_in_five_minutes': len(self.recovery_attempts)}, ensure_ascii=False)))
        if waiting:
            self.publish_state('WAITING_FOR_FAULT_RECOVERY')
        else:
            self.publish_state('FAULT_STOPPED')
        if not ready:
            return False
        self.publish_state('AUTO_REENABLING')
        self.recovery_attempts.append(now)
        self.recovery_dwell.reset()
        try:
            # Recheck callbacks immediately before reset; the gate independently
            # checks first fault, manual stop, fresh health, zero output and pose.
            with self.lock:
                if self.stop_requested or self.recovery_block:
                    return False
            response = self.auto_reset_service()
            if not response.success:
                rospy.logwarn('Automatic fault reset refused: %s', response.message)
                return False
            deadline = time.monotonic()+2.0
            while self.latched and time.monotonic() < deadline and not self.stop_requested:
                time.sleep(.02)
            if (self.latched or self.stop_requested or self.recovery_block
                    or not self.safety_healthy or not self.data_ready(time.monotonic())):
                raise RuntimeError('recovery reset acknowledgement or health lost')
            if not self.request_arm(time.monotonic(), recovery=True):
                return False
            deadline = time.monotonic()+2.0
            while not self.armed and time.monotonic() < deadline and not self.stop_requested:
                time.sleep(.02)
            if not self.armed or self.stop_requested or self.recovery_block:
                raise RuntimeError('recovery arm acknowledgement lost or cancelled')
        except (rospy.ServiceException, RuntimeError) as error:
            self.recovery_block = 'recovery_failed: '+str(error)
            self.disable_bridge()
            return False
        self.paused = False
        self.resume_ready_since = self.empty_since = None
        self.handled_obstacle_event = self.obstacle_event
        self.idle_since = time.monotonic()
        self.result = 'STOPPED'
        self.recovery_publisher.publish(String(data='recovered: retained map; waiting for a new planned goal'))
        self.publish_state('WAITING_FOR_GOAL')
        return True

    def hold_obstacle(self):
        if self.obstacle_event != self.handled_obstacle_event:
            self.handled_obstacle_event = self.obstacle_event
            if self.selected_goal is not None:
                goal = self.selected_goal
                self.recovery_blacklist.append(dict(
                    x=float(goal.pose.position.x), y=float(goal.pose.position.y),
                    frame=goal.header.frame_id, until=rospy.Time.now().to_sec()+30.0))
        # Gate already paused and cancelled. No reset/arm service is used.
        self.hold_session(fault=False, obstacle=True)

    def resume_if_ready(self, now):
        if not self.paused:
            return True
        if not self.can_run(now) or self.active:
            self.resume_ready_since = None
            return False
        if self.resume_ready_since is None:
            self.resume_ready_since = now
        if now - self.resume_ready_since < 2.0:
            return False
        try:
            response = self.resume_service()
        except rospy.ServiceException:
            return False
        if response.success:
            self.paused = False
            self.resume_ready_since = None
            self.idle_since = now
            return True
        return False

    def finish(self):
        # Stop/cancel first, before stopping the goal producer or saving a snapshot.
        try:
            self.stop_service()
        except rospy.ServiceException as error:
            rospy.logwarn('Stop service unavailable during shutdown: %s', error)
        self.disable_bridge()
        self.stop_child()
        save_status='shutdown_save_pending'
        if not rospy.is_shutdown():
            try:
                saved = self.save_service()
                save_status='saved' if saved.success else 'failed'
                if not saved.success:
                    self.result = 'SAVE_FAILED'
                    rospy.logerr('Static point-cloud snapshot failed: %s', saved.message)
            except rospy.ServiceException as error:
                save_status='failed'
                self.result = 'SAVE_FAILED'
                rospy.logerr('Static point-cloud snapshot unavailable: %s', error)
        # Ctrl+C also shuts the original mapper down; its normal final save remains.
        if self.session_dir:
            directory=Path(self.session_dir)
            directory.mkdir(parents=True,exist_ok=True)
            temporary=directory/'result.json.tmp'
            temporary.write_text(json.dumps({'result':self.result,'map_name':self.map_name,
                'real_sdk':self.use_real_sdk,'save_status':save_status,'wall_time':time.time()},indent=2))
            temporary.replace(directory/'result.json')
        rospy.loginfo('Session %s. Export after exit: run_go2_explore export-map %s', self.result, self.map_name)
        if not rospy.is_shutdown():
            self.publish_state(self.result)

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.stop_requested:
                    self.result = 'STOPPED'
                    break
                with self.lock:
                    now = time.monotonic()
                    lost_gate = self.ever_armed and not self.armed
                    failed = (self.startup_failed or
                              (self.bridge_ever_enabled and not self.bridge_enabled) or
                              ((self.arm_requested or self.ever_armed)
                              and (self.latched or lost_gate
                                   or (not self.armed and now-self.arm_request_time > 2.0))))
                    stopped = self.latched and self.safety_stamp > 0
                    waiting = ((self.arm_requested or self.ever_armed)
                               and not self.data_ready(now))
                    done = self.completed(now)
                    arm = self.should_arm(now)
                    start = self.child is None and self.can_start(now)
                    restart = self.child is not None and self.can_restart(now)
                    active, available = self.active, self.available
                if failed or stopped:
                    self.result = 'STOPPED' if self.safety_reason == 'operator_stop' else 'FAULT'
                    rospy.logwarn_throttle(5.0, 'Exploration parked; mapping and diagnostics retained: %s', self.safety_reason)
                    self.hold_session(fault=True)
                    self.recover_if_ready(time.monotonic())
                    time.sleep(0.2)
                    continue
                if self.obstacle_hold or self.obstacle_event != self.handled_obstacle_event:
                    self.hold_obstacle()
                    time.sleep(0.2)
                    continue
                if waiting:
                    self.hold_session(fault=False)
                    time.sleep(0.2)
                    continue
                if not self.resume_if_ready(now):
                    if self.handled_obstacle_event and self.paused:
                        self.publish_state('RECOVERING_AFTER_OBSTACLE')
                    time.sleep(0.2)
                    continue
                if done:
                    self.result = 'COMPLETED'
                    rospy.loginfo('No qualifying reachable frontiers for %.1f s across %d new maps',
                                  self.completion_hold, self.completion_maps)
                    break
                if arm:
                    self.request_arm(now)
                if start:
                    self.idle_since = time.monotonic()
                    self.start()
                elif restart:
                    self.stop_child()
                    with self.lock:
                        if self.can_restart(time.monotonic()):
                            self.restarts += 1
                            self.idle_since = time.monotonic()
                            self.start()
                if self.child is not None:
                    self.publish_state('EXPLORING' if active else
                                       'WAITING_FOR_GOAL' if available else 'CONFIRMING_COMPLETE')
                elif not self.auto_start and not self.ever_armed:
                    self.publish_state('WAITING_FOR_ARM')
                elif self.empty_since is not None:
                    self.publish_state('CONFIRMING_COMPLETE')
                else:
                    self.publish_state('WAITING_FOR_READY')
                time.sleep(0.2)
        finally:
            self.finish()


if __name__ == '__main__':
    rospy.init_node('go2_explore_supervisor')
    Supervisor().run()
