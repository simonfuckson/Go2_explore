#!/usr/bin/env python3
"""Compare two real selectors on one frozen map, with no motion transport."""
import argparse,copy,fcntl,json,os,shlex,signal,subprocess,time,uuid
from pathlib import Path

WS=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--baseline-binary',type=Path,required=True)
    parser.add_argument('--candidate-binary',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=18.)
    args=parser.parse_args()
    from session import LOCK,conflicts
    lock=LOCK.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if conflicts() or (WS/'.state/session.json').exists():raise RuntimeError('Stop the robot session before replay')
    directory=WS/'artifacts'/('selection_replay_'+uuid.uuid4().hex[:8]);directory.mkdir()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11327',ROS_IP='127.0.0.1',ROS_LOG_DIR=str(directory/'ros'))
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','global_planner','costmap_2d','roscpp'],text=True))
    subprocess.run(['g++','-O2','-std=c++14',str(WS/'src/go2_explore_lite/test/snapshot_plan_server.cpp'),
                    *flags,'-o',str(directory/'planner')],check=True)
    processes=[];logs=[]
    def launch(command,name):
        log=(directory/(name+'.log')).open('w');logs.append(log)
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    result={'snapshot':args.snapshot.name,'real_motion_tested':False,'runs':{}}
    try:
        launch(['roscore','-p','11327'],'core')
        import rospy,rosgraph,tf2_ros,actionlib
        from std_msgs.msg import String
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import TransformStamped,PoseStamped,Twist
        from move_base_msgs.msg import MoveBaseAction
        deadline=time.monotonic()+12
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('Replay master timeout')
            time.sleep(.1)
        rospy.init_node('verify_selection_snapshot',disable_signals=True)
        metadata=json.loads((args.snapshot/'metadata.json').read_text())
        for name,value in metadata['parameters'].items():rospy.set_param(name,value)
        rospy.set_param('/snapshot_planner/GlobalPlanner',metadata['parameters']['/move_base']['GlobalPlanner'])
        rospy.set_param('/exploration/runtime_mode','simulation')
        rospy.set_param('/explore/preview_only',True)
        rospy.set_param('/explore/recovery_blacklist',[])
        # A connected action server is required even for preview-only selection.
        action=actionlib.SimpleActionServer('/move_base',MoveBaseAction,auto_start=False);action.start()
        topics={}
        for topic,info in metadata['topics'].items():
            if info['type']=='nav_msgs/OccupancyGrid':
                msg=OccupancyGrid().deserialize((args.snapshot/info['file']).read_bytes())
                topics[topic]=(rospy.Publisher(topic,OccupancyGrid,queue_size=1,latch=True),msg)
        transforms=[TransformStamped().deserialize((args.snapshot/name).read_bytes())
                    for name in ('map__base_link.tfmsg','odom__map.tfmsg')]
        broadcaster=tf2_ros.TransformBroadcaster()
        def publish(_=None):
            stamp=rospy.Time.now()
            for pub,msg in topics.values():msg.header.stamp=stamp;pub.publish(msg)
            for msg in transforms:msg.header.stamp=stamp
            broadcaster.sendTransform(transforms)
        timer=rospy.Timer(rospy.Duration(.10),publish);publish()
        statuses=[];goals=[];commands=[]
        subs=[rospy.Subscriber('/explore/selection_status',String,lambda msg:statuses.append((time.monotonic(),msg.data)),queue_size=50),
              rospy.Subscriber('/explore/selected_goal',PoseStamped,lambda msg:goals.append(msg),queue_size=20),
              rospy.Subscriber('/cmd_vel_safe',Twist,lambda msg:commands.append(msg),queue_size=10)]
        server=launch([str(directory/'planner'),str(args.snapshot/'move_base__global_costmap__costmap.msg')],'planner')
        rospy.wait_for_service('/move_base/make_plan',timeout=10)
        for name,binary,frequency in [('baseline',args.baseline_binary,.5),('candidate',args.candidate_binary,1.)]:
            rospy.set_param('/explore/planner_frequency',frequency)
            rospy.set_param('/explore/selection_budget_ms',75.)
            rospy.set_param('/explore/goal_overhead_seconds',2.)
            statuses.clear();goals.clear();start=time.monotonic()
            process=launch([str(binary),'__name:=explore'],name)
            while time.monotonic()-start<args.seconds:
                if process.poll() is not None:raise RuntimeError(name+' selector exited')
                time.sleep(.1)
            stop(process)
            run={'statuses':list(statuses),'selected_goals':len(goals)}
            run['status_count']=len(statuses)
            run['checked']=sum(int(dict(t.split('=',1) for t in text.split() if '=' in t).get('checked',0)) for _,text in statuses)
            if not statuses:raise AssertionError(name+' did not select/check any candidates')
            result['runs'][name]=run
        assert not commands,'Offline replay unexpectedly had a command publisher'
        result['no_motion_commands']=True;result['passed']=True
    except Exception as exc:
        result.update(passed=False,error=repr(exc))
    finally:
        for process in reversed(processes):stop(process)
        for log in logs:log.close()
        (directory/'verification.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({'directory':str(directory),**{k:v for k,v in result.items() if k!='runs'},
                         'runs':{k:{x:y for x,y in v.items() if x!='statuses'} for k,v in result['runs'].items()}}),flush=True)
    return 0 if result.get('passed') else 1

if __name__=='__main__':raise SystemExit(main())
