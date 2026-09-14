#!/usr/bin/env python3
"""Assert the entire ROS planning/control chain using only mock SDK transport."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xmlrpc.client

WS=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=11322)
    parser.add_argument('--observe-seconds',type=float,default=3.,help='Steady mock motion observation before injecting the fault')
    parser.add_argument('--fault',choices=('odom','terrain','cloud','obstacle','tf','raw','map','takeover','node_exit'),default='odom')
    args=parser.parse_args()
    from session import LOCK,conflicts
    stack_lock=LOCK.open('a')
    fcntl.flock(stack_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if conflicts(): raise RuntimeError('Stop other robot stacks before simulation')
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%args.port
    os.environ['ROS_IP']='127.0.0.1'
    directory=WS/'artifacts'/('simulation_'+args.fault+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    os.environ['ROS_LOG_DIR']=str(directory/'ros')
    import rospy
    import rosgraph
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist,PoseStamped
    from std_msgs.msg import Bool,String
    from std_srvs.srv import Trigger,SetBool
    log=(directory/'console.log').open('wb')
    child=subprocess.Popen(['roslaunch','-p',str(args.port),'go2_exploration','mock_simulation.launch',
        'map_name:='+directory.name,'session_dir:='+str(directory),'auto_reenable:=false',
        'map_root:='+str(WS/'artifacts/simulation_maps')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    commands=[];states=[];goals=[];enabled=[];diagnostics=[]
    result={'fault':args.fault,'directory':str(directory)}
    paused_pid=None
    def await_condition(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if child.poll() is not None: raise RuntimeError('ROS launch exited early')
            if predicate(): return
            time.sleep(.05)
        raise AssertionError('condition timed out after %.1f s'%seconds)
    try:
        await_condition(lambda:rosgraph.is_master_online(),15)
        rospy.init_node('verify_exploration',anonymous=True,disable_signals=True)
        subs=[rospy.Subscriber('/cmd_vel_safe',Twist,lambda m:commands.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),queue_size=100),
              rospy.Subscriber('/exploration/state',String,lambda m:states.append((time.monotonic(),m.data)),queue_size=20),
              rospy.Subscriber('/explore/selected_goal',PoseStamped,lambda m:goals.append((m.pose.position.x,m.pose.position.y)),queue_size=20),
              rospy.Subscriber('/go2/control/enabled',Bool,lambda m:enabled.append((time.monotonic(),m.data)),queue_size=20),
              rospy.Subscriber('/go2_exploration_safety/status',DiagnosticArray,lambda m:diagnostics.append(m),queue_size=5)]
        await_condition(lambda:any(v>.04 for _,v,_,_ in commands),60)
        start=time.monotonic()
        await_condition(lambda:time.monotonic()-start>args.observe_seconds,args.observe_seconds+3)
        assert goals,'no selected exploration goal'
        assert not any(s=='FAULT_STOPPED' for _,s in states),'unexpected fault before fault injection'
        assert any(v for _,v in enabled),'bridge was never automatically enabled'
        assert all(-1e-4<=v<=.3001 and abs(y)<1e-8 and abs(w)<=.5001 for _,v,y,w in commands),'unsafe velocity'
        fault_time=time.monotonic()
        rospy.set_param('/simulation/fault',args.fault)
        if args.fault in ('raw','map','node_exit'):
            node={'raw':'/move_base','map':'/go2_observed_map_memory','node_exit':'/go2_velocity_shaper'}[args.fault]
            uri=rosgraph.Master(rospy.get_name()).lookupNode(node)
            code,message,paused_pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
            assert code==1
            assert str(WS) in Path('/proc/%d/cmdline'%paused_pid).read_bytes().decode() or node=='/move_base'
            os.kill(paused_pid,signal.SIGINT if args.fault=='node_exit' else signal.SIGSTOP)
            if args.fault=='node_exit': paused_pid=None
        elif args.fault=='takeover':
            assert rospy.ServiceProxy('/go2_sdk_bridge_mock/enable',SetBool)(False).success
        if args.fault=='node_exit':
            child.wait(timeout=45)
            assert any(t>fault_time and abs(v)+abs(y)+abs(w)<1e-8 for t,v,y,w in commands),'no stop after node exit'
            assert all(abs(v)+abs(y)+abs(w)<1e-8 for t,v,y,w in commands if t>fault_time+1.2),'late motion after node exit'
            result['node_exit_stopped_session']=True
        elif args.fault=='obstacle':
            await_condition(lambda:any(s=='WAITING_FOR_OBSTACLE' and t>=fault_time for t,s in states),5)
            clear_time=time.monotonic()
            rospy.set_param('/simulation/fault','')
            await_condition(lambda:any(t>clear_time+4 and v>.04 for t,v,_,_ in commands),25)
            result['obstacle_recovery']=True
        else:
            await_condition(lambda:any(not v and t>=fault_time for t,v in enabled),4)
            stop_deadline=fault_time+(3.0 if args.fault=='map' else 1.2)
            if paused_pid:
                os.kill(paused_pid,signal.SIGCONT);paused_pid=None
            rospy.set_param('/simulation/fault','')
            end=time.monotonic()+5
            await_condition(lambda:time.monotonic()>=end,7)
            assert all(abs(v)+abs(y)+abs(w)<1e-8 for t,v,y,w in commands if t>stop_deadline),'motion resumed after fault'
            assert not any(v and t>stop_deadline for t,v in enabled),'automatic re-enable after fault'
            result['fault_stop_and_no_rearm']=True
        if args.fault!='node_exit':
            rospy.wait_for_service('/exploration/stop',timeout=3)
            stopped=rospy.ServiceProxy('/exploration/stop',Trigger)()
            assert stopped.success
            child.wait(timeout=45)
        summary=json.loads((directory/'result.json').read_text())
        if args.fault=='node_exit':
            assert summary['result'] in ('FAULT','STOPPED') and summary['save_status']=='shutdown_save_pending',summary
        else:
            assert summary['result']=='STOPPED' and summary['save_status']=='saved',summary
        result.update(passed=True,goal_count=len(goals),command_count=len(commands),
                      nonzero_commands=sum(abs(v)+abs(w)>1e-4 for _,v,_,w in commands),
                      max_forward=max(v for _,v,_,_ in commands),max_yaw=max(abs(w) for _,_,_,w in commands),
                      states=[s for _,s in states],stop_result=summary)
    except Exception as error:
        result.update(passed=False,error=repr(error),states=[s for _,s in states],goal_count=len(goals),
                      command_count=len(commands))
        if diagnostics:
            result['last_safety']=[{v.key:v.value for v in s.values} for s in diagnostics[-1].status]
    finally:
        if paused_pid:
            try: os.kill(paused_pid,signal.SIGCONT)
            except ProcessLookupError: pass
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try: child.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        log.close()
        (directory/'verification.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
