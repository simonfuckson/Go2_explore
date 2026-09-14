#!/usr/bin/env python3
"""Read-only RViz status feed and compact first-failure evidence log."""
import json
import math
import os
from pathlib import Path
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import Bool, String

STATES={'WAITING_FOR_READY':'等待感知和地图就绪','WAITING_FOR_ARM':'观察模式：不自动运动',
        'WAITING_FOR_POSTURE_STABILITY':'SDK 已使能，等待姿态和感知稳定',
        'WAITING_FOR_GOAL':'等待可执行探索目标','EXPLORING':'正在探索',
        'FAULT_STOPPED':'锁停：查看故障及自动恢复原因','WAITING_FOR_OBSTACLE':'停车等待：检查可行恢复方向',
        'WAITING_FOR_FAULT_RECOVERY':'故障停车：等待数据正常及停稳后自动恢复',
        'AUTO_REENABLING':'自动重新使能：保持零速度，等待姿态稳定',
        'RECOVERING_AFTER_OBSTACLE':'障碍已清除，等待重新选点',
        'PAUSED_WAITING_FOR_DATA':'数据暂停','CONFIRMING_COMPLETE':'正在确认探索完成',
        'COMPLETED':'探索完成','STOPPED':'已停止'}

def motion_evidence(values):
    if values.get('no_step_response')=='true':return '非零指令下未检测到足够运动响应，已触发保护',2
    try:age=float(values.get('last_nonzero_command_age_sec','nan'))
    except ValueError:age=float('nan')
    if not math.isfinite(age):return '无真实运动遥测',3
    if age<0:return 'SDK 尚未收到非零指令；无法判断是否迈步',1
    return '已有非零指令；足端卸载距今 '+values.get('last_foot_unload_age_sec','?')+' s（-1 为未检测到）',1


def selection_explanation(status):
    fields=dict(token.split('=',1) for token in str(status).split() if '=' in token)
    if str(status).startswith('SELECTED'):
        return ('预计新增观测 '+fields.get('gain_m2','?')+' m² | 路程 '+fields.get('route_m','?')+
                ' m | 选点耗时 '+fields.get('selection_ms','?')+' ms')
    reason=fields.get('reason','')
    labels={'global_footprint':'整条路径的机身范围受阻',
            'local_stop_envelope':'近处转向或停车范围受阻',
            'arrival_rotation':'目标处转身范围受阻',
            'no_global_plan':'全局规划器找不到路径',
            'candidate_search_pending':'继续搜索剩余候选点',
            'no_frontier_tasks':'当前没有前沿任务，等待完成条件确认'}
    text=labels.get(reason,'等待可执行目标')
    if 'waiting_s' in fields:text+=' | 已等待 '+fields['waiting_s']+' s'
    if reason in ('global_footprint','local_stop_envelope','arrival_rotation') and 'blocked_x' in fields:
        text+=' | 阻塞位置 ('+fields['blocked_x']+', '+fields.get('blocked_y','?')+') '+fields.get('blocked_frame','')
    return text


class Dashboard:
    def __init__(self):
        self.lock=threading.RLock();self.latest={};self.first_failure=None;self.saw_enabled=False
        self.started=time.monotonic();self.last_log=0;self.last_event=None
        self.pub=rospy.Publisher('/exploration/dashboard',String,queue_size=1,latch=True)
        self.mode=rospy.get_param('/exploration/runtime_mode','unknown')
        self.mount_pitch=rospy.get_param('/mid360_mount/base_link_to_lidar_link/pitch_deg',None)
        self.trajectory=NavPath()
        self.trajectory_pub=rospy.Publisher('/exploration/robot_path',NavPath,queue_size=1,latch=True)
        self.subs=[]
        self.subs.append(rospy.Subscriber('/odom_robot',Odometry,self.odometry,queue_size=2))
        for topic in ('/terrain/status','/go2_exploration_safety/status','/go2/diagnostics'):
            self.subscribe(topic,DiagnosticArray)
        for topic in ('/exploration/state','/exploration/odom_status','/exploration/map_status',
                      '/exploration/recovery_status',
                      '/exploration/coverage_status','/exploration/frontier_status','/explore/selection_status',
                      '/exploration/clearing_status','/exploration/sensor_status','/exploration/self_filter/status','/move_base/TebLocalPlannerROS/execution_mode'):
            self.subscribe(topic,String)
        self.subscribe('/go2/control/enabled',Bool)
        self.subscribe('/explore/selected_goal',PoseStamped)
        for topic in ('/cmd_vel_nav','/exploration/cmd_vel_shaped','/cmd_vel_safe'):
            self.subscribe(topic,Twist)
        directory=Path(rospy.get_param('~session_dir',str(Path.home()/'go2_explore_ws/logs/direct_launch'))).resolve()
        ws=(Path.home()/'go2_explore_ws').resolve()
        if ws not in directory.parents:raise ValueError('Dashboard log must stay in exploration workspace')
        directory.mkdir(parents=True,exist_ok=True)
        self.log=(directory/'status_events.jsonl').open('a',buffering=1)
        self.timer=rospy.Timer(rospy.Duration(.5),self.tick)
        rospy.on_shutdown(self.close)

    def odometry(self,message):
        # The public body trajectory uses the same startup-normalized odom as
        # the footprint and map. FAST-LIO's optional path tracks the IMU instead.
        p=message.pose.pose.position
        if message.header.frame_id!='odom' or not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
            return
        with self.lock:
            self.latest['/odom_robot']=({'xyz':[p.x,p.y,p.z]},time.monotonic())
            self.trajectory.header=message.header
            previous=self.trajectory.poses[-1].pose.position if self.trajectory.poses else None
            if previous is None or math.dist((p.x,p.y,p.z),(previous.x,previous.y,previous.z))>=.025:
                self.trajectory.poses.append(PoseStamped(header=message.header,pose=message.pose.pose))
                self.trajectory.poses=self.trajectory.poses[-20000:]
            self.trajectory_pub.publish(self.trajectory)

    def subscribe(self,topic,kind):
        self.subs.append(rospy.Subscriber(topic,kind,self.receive,callback_args=topic,queue_size=20))

    def receive(self,message,topic):
        now=time.monotonic()
        if isinstance(message,DiagnosticArray):
            value=[dict(name=s.name,level=s.level,message=s.message,values={v.key:v.value for v in s.values}) for s in message.status]
        elif isinstance(message,Twist):value=[message.linear.x,message.linear.y,message.angular.z]
        elif isinstance(message,PoseStamped):value=[message.pose.position.x,message.pose.position.y,message.header.frame_id]
        else:value=message.data
        with self.lock:
            self.latest[topic]=(value,now)
            if topic=='/go2/control/enabled' and value:self.saw_enabled=True
            if topic=='/go2_exploration_safety/status':
                for status in value:
                    try:context=json.loads(status['values'].get('stop_context_json','null'))
                    except (ValueError,TypeError):continue
                    if not isinstance(context,dict):continue
                    token=(context.get('stamp'),context.get('sequence'))
                    if token!=getattr(self,'last_stop_token',None):
                        self.last_stop_token=token
                        if hasattr(self,'log') and not self.log.closed:
                            self.log.write(json.dumps(dict(event='safety_stop_context',context=context),ensure_ascii=False)+'\n')
            # Capture the full terrain reason at the moment its gate closes,
            # including during SDK preparation. It must survive later recovery.
            if topic=='/terrain/status' and self.saw_enabled and not self.first_failure:
                for s in value:
                    if s['values'].get('health_gate_open')=='false':
                        self.first_failure=dict(time=rospy.Time.now().to_sec(),diagnostic=s)
                        if hasattr(self,'log'):
                            self.log.write(json.dumps(dict(event='first_terrain_failure',**self.first_failure),ensure_ascii=False)+'\n')

    def close(self):
        with self.lock:
            if hasattr(self,'log') and not self.log.closed:self.log.close()

    def snapshot(self):
        now=time.monotonic();rows=[]
        def value(topic,default=None):return self.latest.get(topic,(default,0))[0]
        def age(topic):return now-self.latest.get(topic,(None,0))[1] if topic in self.latest else None
        def diagnostic(topic):
            items=value(topic,[])
            return items[0] if items else dict(message='尚无数据',values={},level=3)
        def row(name,text,topic,level=0,limit=2.5,detail=''):
            a=age(topic)
            if a is None:level=3
            elif limit and a>limit:level=3;text+=' [数据过期]'
            rows.append(dict(name=name,value=str(text),level=level,age=a,detail=detail))
        state=value('/exploration/state','WAITING_FOR_READY')
        gate=diagnostic('/go2_exploration_safety/status');g=gate['values']
        terrain=diagnostic('/terrain/status');t=terrain['values']
        sdk=diagnostic('/go2/diagnostics');s=sdk['values']
        row('安全门 / 停车原因',gate['message'],'/go2_exploration_safety/status',gate['level'],1.,json.dumps(g,ensure_ascii=False,indent=2))
        recovery=('已停稳，前方检查通过，等待重新选点' if g.get('obstacle_replan_ready')=='True' else
                  '旧方向已恢复，等待重新选点' if g.get('obstacle_reason') and g.get('obstacle_hold')=='False' else
                  '正在确认停稳' if g.get('standstill_confirmed')!='True' else
                  '前方可通行，持续确认中' if g.get('replan_candidate_clear')=='True' else
                  '已停稳，前方停车范围仍受阻') if g.get('obstacle_reason') else '无需障碍恢复'
        row('障碍恢复',recovery,'/go2_exploration_safety/status',1 if g.get('obstacle_reason') else 0,1.)
        row('SDK 控制','已使能' if value('/go2/control/enabled',False) else '已禁用','/go2/control/enabled',0 if value('/go2/control/enabled',False) else 1,None)
        recovery_text=value('/exploration/recovery_status','等待自动恢复状态')
        try:
            recovery_data=json.loads(recovery_text)
            recovery_reason={'no_fault':'当前无故障',
                'confirming_stable_recovery':'持续确认健康数据',
                'waiting_for_standstill_and_clear_stop_region':'等待停稳和停车范围可通行',
                'odom_health_unavailable':'里程计尚未恢复',
                'terrain_health_unavailable':'避障感知尚未就绪，详见感知状态',
                'sdk_or_gait_fault':'原 SDK 健康诊断或步态故障未消除',
                'remote_or_posture_override':'遥控或姿态控制接管',
                'automatic_reenable_disabled':'本会话已关闭自动恢复',
                'recovery_cooldown':'等待 30 秒恢复间隔',
                'recovery_rate_limit: three attempts in five minutes':'5 分钟内已尝试 3 次，等待恢复间隔',
                }.get(recovery_data['reason'],recovery_data['reason'])
            recovery_text=('已开启' if recovery_data['enabled'] else '已关闭')+'；'+recovery_reason+\
                '；稳定 %.1f / %.1f 秒'%(recovery_data['stable_seconds'],recovery_data['required_seconds'])
        except (ValueError,KeyError,TypeError):
            if recovery_text.startswith('recovered:'):
                recovery_text='已恢复：保留本次地图，重新规划继续探索'
        row('故障自动恢复',recovery_text,'/exploration/recovery_status',1,None)
        body=value('/odom_robot')
        row('机身坐标（odom）',('x %.3f / y %.3f / z %.3f m'%tuple(body['xyz'])) if body else '尚无机身位姿','/odom_robot')
        pitch=getattr(self,'mount_pitch',None)
        rows.append(dict(name='坐标轴说明',value='红 X 前 / 绿 Y 左 / 蓝 Z 上；map 零点为启动机身中心。'+('雷达俯倾 %.1f°。'%pitch if pitch is not None else ''),level=0,age=None,detail='地面通常位于 z<0；lio_odom/body_lio 是估计器坐标，不是机身坐标。'))
        row('里程计 / TF',str(value('/exploration/odom_status','尚无数据'))+' | TF '+g.get('tf_age_s','?')[:6]+' s','/exploration/odom_status')
        sensor=str(value('/exploration/sensor_status','尚无真实雷达监测'))
        row('雷达网络 / 数据连续性',sensor,'/exploration/sensor_status',2 if sensor.startswith('FAULT:') else 0, .3,
            'eth1 为 MID360 网络。断流后停止向 FAST-LIO 输入，修复网线/供电后重启会话；不会自动恢复运动。')
        row('地面网格清空',str(value('/exploration/clearing_status','尚无数据')),'/exploration/clearing_status')
        mask=str(value('/exploration/self_filter/status','模拟场景无硬件自身过滤'))
        mask_text=('未启用：等待相机和支架安装范围标定' if mask.startswith('disabled:') else
                   '仅预览，不剔除点云：'+mask[len('preview only: '):] if mask.startswith('preview only:') else
                   '已启用：'+mask[len('active: '):] if mask.startswith('active:') else
                   '过滤故障：'+mask[len('FAULT: '):] if mask.startswith('FAULT:') else mask)
        row('相机与支架过滤',mask_text,'/exploration/self_filter/status',2 if mask.startswith('FAULT:') else 0, .5,mask)
        execution=value('/move_base/TebLocalPlannerROS/execution_mode','等待规划器')
        execution_text={'FORWARD_KNOWN_ROUTE_CONNECTION':'沿已观测通道前进，连接当前路线',
                        'LOCAL_STOP_ENVELOPE_BLOCKED':'转动/停车范围受阻，等待其他可行路线',
                        'ALIGN_TO_PATH':'原地对准路径','TEB_FORWARD':'沿 TEB 路径前进'}.get(execution,execution)
        row('规划执行方式',str(execution_text),'/move_base/TebLocalPlannerROS/execution_mode',
            1 if execution=='LOCAL_STOP_ENVELOPE_BLOCKED' else 0,limit=None,detail=str(execution))
        flat=t.get('ground_geometry_checks')=='disabled'
        row('避障感知' if flat else '地形健康',
            '感知就绪：地形评分已关闭，避障检查有效' if flat and terrain['level']==0 else terrain['message'],
            '/terrain/status',terrain['level'],1.,json.dumps(t,ensure_ascii=False,indent=2))
        row('感知数据' if flat else '地形数据','频率 '+t.get('output_rate_hz','?')+' Hz | '+
            ('平地探索模式' if flat else '高度 '+t.get('estimated_sensor_height_m','?')+' m'),'/terrain/status',terrain['level'])
        row('地面支撑（仅显示）' if flat else '地面支撑','近场 '+t.get('near_support_area_m2','?')+' m² | 连通 '+t.get('connected_ground_area_m2','?')+' m²',
            '/terrain/status',0 if flat else terrain['level'])
        row('地图更新',value('/exploration/map_status','尚无数据'),'/exploration/map_status')
        row('地面覆盖',value('/exploration/coverage_status','尚无数据'),'/exploration/coverage_status')
        row('前沿任务',value('/exploration/frontier_status','尚无数据'),'/exploration/frontier_status')
        selection=value('/explore/selection_status','选点器未发布；停车后会停止选点')
        row('选点结果',selection,'/explore/selection_status',0 if str(selection).startswith('SELECTED') else 1,6.)
        row('探索效率 / 等待原因',selection_explanation(selection),'/explore/selection_status',
            0 if str(selection).startswith('SELECTED') else 1,6.,str(selection))
        target=value('/explore/selected_goal')
        row('最近选定目标',('x %.2f / y %.2f m (%s)'%tuple(target)) if target else '尚无目标','/explore/selected_goal',0,None)
        row('当前执行目标 / TEB','活动目标 '+g.get('move_base_status_live_goal_count','0')+' | 轨迹 '+g.get('local_plan_poses','0')+' 点','/go2_exploration_safety/status')
        recovery_reason=g.get('auto_recovery_reason','')
        recovery_text={'not_latched':'当前未锁停','ready':'停稳与停车范围检查通过',
            'recovery_inhibited':'当前故障禁止自动恢复','inputs_changed_during_check':'输入更新，正在重新检查',
            'input_health_not_ready':'感知或定位数据未就绪',
            'stationary_or_forward_stop_region_blocked':'当前机身或前方停车范围仍受阻',
            'waiting_for_control_disable':'等待控制禁用','waiting_for_goal_cancel':'等待目标取消',
            'waiting_for_shaper_zero':'等待整形速度归零','move_base_status_stale':'导航状态过期',
            'waiting_for_zero_output':'等待运动输出归零','stationary_tf_stale':'停稳判定所需 TF 过期',
            'waiting_for_measured_standstill':'等待位姿连续确认停稳',
            'collision_check_budget_exhausted':'本轮检查未完成，保持停车并重查'}.get(recovery_reason,recovery_reason)
        row('自动恢复准入',recovery_text or '等待数据','/go2_exploration_safety/status',
            0 if recovery_reason in ('ready','not_latched') else 1)
        row('规划停车请求','已要求停车，安全输出立即归零' if g.get('raw_planner_stop')=='True' else '无停车请求',
            '/go2_exploration_safety/status')
        for topic,label in (('/cmd_vel_nav','规划速度'),('/exploration/cmd_vel_shaped','整形速度'),('/cmd_vel_safe','安全输出速度')):
            v=value(topic)
            row(label,('前进 %.3f / 横移 %.3f m/s / 转向 %.3f rad/s'%tuple(v)) if v else '尚无速度指令',topic,0,.6)
        row('SDK 实际请求','前进 '+s.get('commanded_vx_m_s','?')+' m/s | 转向 '+s.get('commanded_wz_rad_s','?')+' rad/s','/go2/diagnostics',sdk['level'])
        row('步态请求',s.get('gait_mode','?')+' | 请求回执 '+s.get('classic_sdk_result','?')+'；实际步态未验证','/go2/diagnostics',max(sdk['level'],1),2.5,json.dumps(s,ensure_ascii=False,indent=2))
        evidence,level=motion_evidence(s)
        row('实际运动证据',evidence,'/go2/diagnostics',level)
        row('电池 / 遥控接管',s.get('battery_soc_percent','?')+' % | 摇杆 '+s.get('remote_axis_max','?')+' | 按键 '+s.get('remote_buttons','?'),'/go2/diagnostics',sdk['level'])
        disk=os.statvfs(str(Path.home()/'go2_explore_ws'));free=disk.f_bavail*disk.f_frsize/(1024**3)
        rows.append(dict(name='磁盘剩余',value='%.2f GiB%s'%(free,'；不足 1 GiB，完整录包会停止' if free<1 else ''),level=2 if free<1 else 0,age=0,detail=''))
        if self.first_failure:
            reason=self.first_failure['diagnostic']['message']
        else:reason='本监视器启动后尚未捕获；已停车的历史原因请看会话日志'
        rows.append(dict(name='感知首次失败原因' if flat else '地形首次失败原因',value=reason,level=2 if self.first_failure else 1,age=None,detail=json.dumps(self.first_failure,ensure_ascii=False,indent=2)))
        mode=getattr(self,'mode','unknown')
        label={'simulation':'全虚拟模拟场景','observe':'真实传感器 · 模拟底盘','real':'真机自主探索'}.get(mode,'运行模式未确认')
        return dict(state=state,mode=mode,title=label+'\n'+STATES.get(state,state),rows=rows,stamp=rospy.Time.now().to_sec())

    def tick(self,_event):
        with self.lock:
            result=self.snapshot()
            self.pub.publish(String(data=json.dumps(result,ensure_ascii=False)))
            event=(result['state'],self.latest.get('/explore/selection_status',(None,0))[0],
                   self.latest.get('/go2/control/enabled',(None,0))[0])
            now=time.monotonic()
            if not self.log.closed and (event!=self.last_event or now-self.last_log>5):
                self.log.write(json.dumps(result,ensure_ascii=False)+'\n');self.last_log=now;self.last_event=event


if __name__=='__main__':
    rospy.init_node('go2_exploration_dashboard')
    Dashboard();rospy.spin()
