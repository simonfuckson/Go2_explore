#!/usr/bin/env python3
"""Exercise real TEB goal arrivals and shaper tails using only the mock SDK."""
import fcntl,json,os,signal,subprocess,time,uuid
from pathlib import Path
WS=Path(__file__).resolve().parents[1]

def main():
    from session import LOCK,active_session,conflicts
    lock=LOCK.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert not active_session() and not conflicts(),'Stop all robot sessions first'
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11322',ROS_IP='127.0.0.1')
    out=WS/'artifacts'/('goal_transitions_'+uuid.uuid4().hex[:8]);out.mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    log=(out/'console.log').open('w')
    child=subprocess.Popen(['roslaunch','-p','11322','go2_exploration','mock_simulation.launch',
        'map_name:='+out.name,'map_root:='+str(WS/'artifacts/simulation_maps'),
        'session_dir:='+str(out),'auto_reenable:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    import rospy,rosgraph,rosnode
    from actionlib_msgs.msg import GoalStatusArray,GoalStatus
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    completed=set();gate={};events=[];enabled=[];raw=[None];tails=[];outputs=[];faults=[]
    result={'real_sdk':False,'directory':str(out)}
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while not predicate():
            assert child.poll() is None,'ROS launch exited'
            assert not faults,dict(faults=faults,gate=gate)
            assert time.monotonic()<end,dict(timeout=seconds,completed=list(completed),gate=gate)
            time.sleep(.05)
    def raw_command(message):
        now=time.monotonic();stopping=abs(message.linear.x)+abs(message.angular.z)<1e-4
        previous=raw[0]
        since=previous[2] if previous is not None and previous[1]==stopping else now
        raw[0]=(now,stopping,since);events.append((now,'raw',message.linear.x,message.angular.z))
    def shaped(message):
        now=time.monotonic()
        if raw[0] and raw[0][1] and now-raw[0][0]<.2 and abs(message.linear.x)+abs(message.angular.z)>.03:
            tails.append((now,message.linear.x,message.angular.z))
    def output(message):
        now=time.monotonic();outputs.append((now,message.linear.x,message.linear.y,message.angular.z))
        if raw[0] and raw[0][1] and now-raw[0][0]<.2 and now-raw[0][2]>.15:
            if abs(message.linear.x)+abs(message.angular.z)>1e-5:faults.append('Motion after fresh planner stop')
    def status(message):
        for item in message.status_list:
            if item.status==GoalStatus.SUCCEEDED:completed.add(item.goal_id.id)
    def diagnostic(message):
        if message.status:
            gate.clear();gate.update({v.key:v.value for v in message.status[0].values})
            if gate.get('latched_stop')=='True':faults.append(gate.get('latched_reason'))
    try:
        wait(rosgraph.is_master_online,15)
        rospy.init_node('verify_goal_transitions',anonymous=True,disable_signals=True)
        rospy.Subscriber('/cmd_vel_nav',Twist,raw_command,queue_size=100)
        rospy.Subscriber('/exploration/cmd_vel_shaped',Twist,shaped,queue_size=100)
        rospy.Subscriber('/cmd_vel_safe',Twist,output,queue_size=100)
        rospy.Subscriber('/move_base/status',GoalStatusArray,status,queue_size=50)
        rospy.Subscriber('/go2_exploration_safety/status',DiagnosticArray,diagnostic,queue_size=20)
        rospy.Subscriber('/go2/control/enabled',Bool,lambda m:enabled.append((time.monotonic(),m.data)),queue_size=20)
        wait(lambda:'/go2_sdk_bridge_mock' in rosnode.get_node_names(),15)
        assert '/go2_sdk_bridge_real' not in rosnode.get_node_names()
        wait(lambda:len(completed)>=3,100)
        assert tails,'No planner-stop/nonzero-shaper-tail transition was exercised'
        assert any(v>.04 for _,v,_,_ in outputs),'No mock walking'
        assert all(-1e-4<=v<=.3001 and abs(y)<1e-8 and abs(w)<=.5001 for _,v,y,w in outputs)
        result.update(passed=True,completed_goals=len(completed),stop_tail_samples=len(tails),
                      no_stop_transition_faults=True,motion_commands=len(outputs),max_forward=max(v for _,v,_,_ in outputs))
        assert rospy.ServiceProxy('/exploration/stop',Trigger)().success
        child.wait(timeout=45)
        saved=json.loads((out/'result.json').read_text());assert saved['save_status']=='saved'
        result['stop_result']=saved
    except Exception as error:
        result.update(passed=False,error=repr(error),gate=gate,completed_goals=len(completed),stop_tail_samples=len(tails))
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=40)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        log.close();(out/'verification.json').write_text(json.dumps(result,indent=2))
        (out/'command_events.json').write_text(json.dumps({'raw':events,'tails':tails,'outputs':outputs}))
        print(json.dumps(result,indent=2),flush=True)
    return 0 if result.get('passed') else 1

if __name__=='__main__':raise SystemExit(main())
