#!/usr/bin/env python3
"""Full ROS planner/shaper/gate/supervisor regression, mock SDK only."""
import argparse,fcntl,json,os,signal,subprocess,time,uuid,xmlrpc.client
from pathlib import Path
WS=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--case',choices=('terrain','raw','manual_after_terrain','odom_continuity','disabled_option','takeover'),default='terrain')
    args=p.parse_args()
    from session import LOCK,active_session,conflicts
    lock=LOCK.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert not active_session() and not conflicts(),'A robot session is active'
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:11324';os.environ['ROS_IP']='127.0.0.1'
    out=WS/'artifacts'/('auto_reenable_'+args.case+'_'+uuid.uuid4().hex[:8]);out.mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    fixture=out/'test.launch'
    fixture.write_text('''<launch>
<include file="$(find go2_exploration)/launch/mock_simulation.launch">
<arg name="map_name" value="%s"/><arg name="map_root" value="%s"/><arg name="session_dir" value="%s"/>
</include><param name="/go2_explore_supervisor/auto_reenable" value="%s"/>
</launch>'''%(out.name,WS/'artifacts/simulation_maps',out,'false' if args.case=='disabled_option' else 'true'))
    log=(out/'console.log').open('wb')
    launch=subprocess.Popen(['roslaunch','-p','11324',str(fixture)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    import rospy,rosgraph,rosnode
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist,PoseStamped
    from std_msgs.msg import Bool,String
    from std_srvs.srv import Trigger,SetBool
    from nav_msgs.msg import Odometry
    commands=[];states=[];goals=[];enabled=[];gate={};recovery=[];paused_pid=None
    result={'case':args.case,'real_sdk':False,'directory':str(out)}
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while not predicate():
            assert launch.poll() is None,'Launch exited early'
            assert time.monotonic()<deadline,dict(timeout=seconds,gate=gate,recovery=recovery[-1:],states=states[-4:])
            time.sleep(.05)
    def dwell(seconds):
        deadline=time.monotonic()+seconds;wait(lambda:time.monotonic()>=deadline,seconds+1)
    def diag(m):
        if m.status:gate.clear();gate.update({v.key:v.value for v in m.status[0].values})
    try:
        wait(rosgraph.is_master_online,15)
        rospy.init_node('verify_automatic_reenable',anonymous=True,disable_signals=True)
        names=rosnode.get_node_names()
        wait(lambda:'/go2_sdk_bridge_mock' in rosnode.get_node_names(),15)
        assert '/go2_sdk_bridge_real' not in rosnode.get_node_names()
        rospy.Subscriber('/cmd_vel_safe',Twist,lambda m:commands.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),queue_size=100)
        rospy.Subscriber('/exploration/state',String,lambda m:states.append((time.monotonic(),m.data)),queue_size=30)
        rospy.Subscriber('/explore/selected_goal',PoseStamped,lambda m:goals.append((time.monotonic(),m.pose.position.x,m.pose.position.y)),queue_size=20)
        rospy.Subscriber('/go2/control/enabled',Bool,lambda m:enabled.append((time.monotonic(),m.data)),queue_size=100)
        rospy.Subscriber('/go2_exploration_safety/status',DiagnosticArray,diag,queue_size=10)
        rospy.Subscriber('/exploration/recovery_status',String,lambda m:recovery.append(m.data),queue_size=10)
        wait(lambda:any(v>.04 for _,v,_,_ in commands),60)
        dwell(1)
        fault_time=time.monotonic()
        if args.case=='raw':
            uri=rosgraph.Master(rospy.get_name()).lookupNode('/move_base')
            code,_,paused_pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name());assert code==1
            os.kill(paused_pid,signal.SIGSTOP)
        elif args.case=='takeover':
            assert rospy.ServiceProxy('/go2_sdk_bridge_mock/enable',SetBool)(False).success
        else:
            rospy.set_param('/simulation/fault','terrain')
        wait(lambda:any(t>=fault_time and not value for t,value in enabled),5)
        wait(lambda:gate.get('latched_stop')=='True',3)
        result['first_fault']=gate.get('latched_reason')
        if args.case=='manual_after_terrain':
            assert rospy.ServiceProxy('/go2_exploration_safety/stop',Trigger)().success
        elif args.case=='odom_continuity':
            rospy.set_param('/simulation/fault','odom')
            dwell(.4)
            # Inject a new timestamp with a 10 m jump into the actual health node.
            jump=Odometry();jump.header.frame_id='odom';jump.child_frame_id='base_footprint'
            jump.header.stamp=rospy.Time.now();jump.pose.pose.position.x=10.;jump.pose.pose.orientation.w=1.
            pub=rospy.Publisher('/odom_nav',Odometry,queue_size=1);dwell(.3)
            jump.header.stamp=rospy.Time.now();pub.publish(jump);dwell(.2)
        if paused_pid:os.kill(paused_pid,signal.SIGCONT);paused_pid=None
        rospy.set_param('/simulation/fault','')
        clear_time=time.monotonic()
        should_recover=args.case in ('terrain','raw')
        if should_recover:
            wait(lambda:any(t>clear_time and v for t,v in enabled),45)
            reenabled=min(t for t,v in enabled if t>clear_time and v)
            assert reenabled-clear_time>=10.,'Continuous health dwell was bypassed'
            assert all(abs(v)+abs(y)+abs(w)<1e-8 for t,v,y,w in commands if fault_time+1.2<t<reenabled)
            wait(lambda:any(t>reenabled and v>.04 for t,v,_,_ in commands),30)
            assert any(t>reenabled for t,_,_ in goals),'Old goal reused without replanning'
            result.update(reenabled_after_clear_sec=reenabled-clear_time,new_goal_and_mock_motion=True)
            stop_time=time.monotonic()
            assert rospy.ServiceProxy('/go2_exploration_safety/stop',Trigger)().success
            wait(lambda:gate.get('latched_stop')=='True',3)
            dwell(14)
            assert all(abs(v)+abs(y)+abs(w)<1e-8 for t,v,y,w in commands if t>stop_time+.3)
            assert not any(t>stop_time+.5 and v for t,v in enabled)
            result['operator_stop_after_recovery_stays_stopped']=True
        else:
            dwell(17)
            assert not any(t>clear_time+.5 and v for t,v in enabled),'Forbidden automatic reenable'
            assert all(abs(v)+abs(y)+abs(w)<1e-8 for t,v,y,w in commands if t>fault_time+1.2)
            result['nonrecoverable_or_disabled_remains_stopped']=True
        assert all(-1e-4<=v<=.3001 and abs(y)<1e-8 and abs(w)<=.5001 for _,v,y,w in commands)
        assert rospy.ServiceProxy('/exploration/stop',Trigger)().success
        launch.wait(timeout=45)
        saved=json.loads((out/'result.json').read_text())
        assert saved['save_status']=='saved' and saved['result']=='STOPPED',saved
        result.update(passed=True,stop_result=saved,goal_count=len(goals),command_count=len(commands),
                      states=[s for _,s in states],recovery_status=recovery[-3:])
    except Exception as error:
        result.update(passed=False,error=repr(error),states=[s for _,s in states],gate=gate,recovery=recovery[-5:])
    finally:
        if paused_pid:
            try:os.kill(paused_pid,signal.SIGCONT)
            except ProcessLookupError:pass
        if launch.poll() is None:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=40)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
        log.close();(out/'verification.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
