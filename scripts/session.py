#!/usr/bin/env python3
"""Own exactly one isolated GO2 exploration session and its process group."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import subprocess
import sys
import time
import uuid
from urllib.parse import urlparse

WS=Path(__file__).resolve().parents[1]
STATE=WS/'.state/session.json'
LOCK=Path('/tmp/go2_nav_ws_nvidia.stack.lock')
BINARIES={'fastlio_mapping','livox_ros_driver2_node','go2_sdk_bridge_real_node',
          'go2_sdk_bridge_mock_node','go2_velocity_shaper_node','move_base'}


def atomic_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value,indent=2,ensure_ascii=False))
    tmp.replace(path)


def verify_snapshot(directory):
    try:
        rows=(directory/'mapping_snapshot.sha256').read_text().splitlines()
        entries=dict(row.split(None,1)[::-1] for row in rows)
        if set(entries)!={'public_map.pcd','traversed_path_map.pcd'}:
            return False
        for name,digest in entries.items():
            checksum=hashlib.sha256()
            with (directory/name).open('rb') as stream:
                for chunk in iter(lambda:stream.read(1024*1024),b''): checksum.update(chunk)
            if checksum.hexdigest()!=digest: return False
        return True
    except (OSError,ValueError): return False


def ticks(pid):
    try:
        value=Path('/proc/%d/stat'%pid).read_text()
        fields=value[value.rfind(')')+2:].split()
        return None if fields[0]=='Z' else fields[19]
    except (FileNotFoundError,PermissionError,IndexError):
        return None


def active_session():
    if not STATE.exists():
        return None
    data=json.loads(STATE.read_text())
    if ticks(data['pid']) != data.get('start_ticks'):
        return None
    if str(WS/'scripts/session.py').encode() not in Path('/proc/%d/cmdline'%data['pid']).read_bytes():
        return None
    return data


def descendants(root_pid):
    parents={}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            data=(proc/'stat').read_text()
            parents[int(proc.name)]=int(data[data.rfind(')')+2:].split()[1])
        except (FileNotFoundError,PermissionError,ValueError,IndexError): pass
    owned={root_pid}
    while True:
        added={pid for pid,parent in parents.items() if parent in owned}-owned
        if not added: return owned
        owned.update(added)


def conflicts(own_pids=()):
    result=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            if int(proc.name) in own_pids: continue
            args=(proc/'cmdline').read_bytes().decode(errors='replace').split('\0')
            if any(Path(a).name in BINARIES for a in args if a):
                result.append({'pid':int(proc.name),'command':args[:2]})
        except (FileNotFoundError,PermissionError,ProcessLookupError):
            pass
    return result


def validate_environment():
    for variable in ('CMAKE_PREFIX_PATH','ROS_PACKAGE_PATH','PYTHONPATH','LD_LIBRARY_PATH'):
        if '/go2_nav_ws' in os.environ.get(variable,'') or '/livox_fastlio' in os.environ.get(variable,''):
            raise RuntimeError('Contaminated environment: '+variable)
    for package in ('go2_core','go2_control','go2_mapping','go2_terrain','go2_exploration','go2_explore_lite','livox_ros_driver2'):
        resolved=Path(subprocess.check_output(['rospack','find',package],text=True).strip()).resolve()
        if WS not in resolved.parents:
            raise RuntimeError('Package outside exploration workspace: '+package+' '+str(resolved))


def ros_client():
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node('go2_explore_cli',anonymous=True,disable_signals=True)
    return rospy


def service(name,kind,*args,timeout=3.):
    rospy=ros_client()
    rospy.wait_for_service(name,timeout=timeout)
    return rospy.ServiceProxy(name,kind)(*args)


def stop_motion():
    from std_srvs.srv import Trigger,SetBool
    errors=[]
    for name,kind,args in (('/go2_exploration_safety/stop',Trigger,()),
                           ('/go2_sdk_bridge_real/enable',SetBool,(False,)),
                           ('/go2_sdk_bridge_mock/enable',SetBool,(False,))):
        try:
            response=service(name,kind,*args,timeout=.5)
            if not response.success: errors.append(name+': '+response.message)
        except Exception:
            pass
    return errors


def stop_session():
    data=active_session()
    if not data:
        print('No managed exploration session is running.')
        return 0
    from std_srvs.srv import Trigger
    try:
        response=service('/exploration/stop',Trigger)
        print(response.message,flush=True)
    except Exception:
        stop_motion()
        os.kill(data['pid'],signal.SIGINT)
    deadline=time.monotonic()+90
    while ticks(data['pid'])==data['start_ticks'] and time.monotonic()<deadline:
        time.sleep(.1)
    if ticks(data['pid'])==data['start_ticks']:
        raise RuntimeError('Session is still saving or stopping; see '+data['session_dir'])
    result_path=Path(data['session_dir'])/'session_result.json'
    if result_path.exists():
        result=json.loads(result_path.read_text())
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result.get('success') else 1
    raise RuntimeError('Session exited without a verified result; inspect '+data['session_dir'])


def status():
    data=active_session()
    print('ROS Master: '+os.environ['ROS_MASTER_URI'])
    print(json.dumps(data or {'running':False},ensure_ascii=False,indent=2))
    if not data: return 0
    import rospy
    from std_msgs.msg import String,Bool
    from diagnostic_msgs.msg import DiagnosticArray
    ros_client()
    latest={}
    subscribers=[]
    for topic in ('/terrain/status','/go2_exploration_safety/status','/go2/diagnostics'):
        subscribers.append(rospy.Subscriber(topic,DiagnosticArray,
            lambda msg,t=topic:latest.update({t:[{'name':s.name,'level':s.level,'message':s.message,
                'values':{v.key:v.value for v in s.values}} for s in msg.status]}),queue_size=1))
    for topic,kind in (('/exploration/state',String),('/exploration/odom_status',String),
                       ('/exploration/recovery_status',String),
                       ('/exploration/sensor_status',String),
                       ('/exploration/map_status',String),('/exploration/coverage_status',String),
                       ('/exploration/frontier_status',String),('/go2/control/enabled',Bool),
                       ('/explore/selection_status',String),
                       ('/terrain/healthy',Bool)):
        subscribers.append(rospy.Subscriber(topic,kind,lambda msg,t=topic:latest.update({t:msg.data}),queue_size=1))
    time.sleep(1.5)
    latest.setdefault('/explore/selection_status','unavailable: selector is not publishing')
    print(json.dumps(latest,ensure_ascii=False,indent=2))
    return 0


def prepare_display():
    # Preserve a terminal/remote desktop display. For SSH, use the same user's
    # existing GO2 desktop; never change X access permissions or system config.
    if not os.environ.get('DISPLAY'):
        os.environ['DISPLAY']=':0'
        candidates=(Path('/run/user')/str(os.getuid())/'gdm/Xauthority',Path.home()/'.Xauthority')
        if not os.environ.get('XAUTHORITY'):
            for path in candidates:
                if path.is_file():os.environ['XAUTHORITY']=str(path);break
    try:
        checked=subprocess.run(['xwininfo','-root'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=3)
        if checked.returncode==0:return
    except (OSError,subprocess.TimeoutExpired):pass
    raise RuntimeError('RViz display unavailable. Open a GO2 desktop terminal, or explicitly use --no-rviz for headless operation.')


def show_rviz():
    data=active_session()
    if not data:raise RuntimeError('No managed exploration session for RViz')
    prepare_display()
    import rosgraph
    master=rosgraph.Master('/go2_exploration_viewer_cli')
    def exists(name):
        try:master.lookupNode(name);return True
        except rosgraph.MasterError:return False
    if exists('/go2_exploration_rviz'):
        print('Exploration RViz is already open on '+os.environ['DISPLAY']);return 0
    child=subprocess.Popen(['roslaunch','go2_exploration','exploration_viewer.launch',
        'session_dir:='+data['session_dir'],
        'start_dashboard:='+str(not exists('/go2_exploration_dashboard')).lower()],start_new_session=True)
    try:
        while child.poll() is None:
            current=active_session()
            if not current or current['session_id']!=data['session_id']:break
            time.sleep(.2)
    except KeyboardInterrupt:pass
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    return 0


def run(args):
    validate_environment()
    if args.rviz:prepare_display()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',args.name):
        raise RuntimeError('Map name must contain only letters, digits, underscore and hyphen')
    if args.command=='observe' and args.real:
        raise RuntimeError('observe always uses the mock bridge')
    if not .30<=args.lidar_blind<=.70:
        raise ValueError('lidar-blind must be finite and between 0.30 and 0.70 metres')
    if active_session(): raise RuntimeError('An exploration session is already running')
    lock=LOCK.open('a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise RuntimeError('Original GO2 mapping/navigation/export or exploration owns the stack lock')
    existing=conflicts()
    if existing: raise RuntimeError('Conflicting robot processes: '+json.dumps(existing))
    port=urlparse(os.environ['ROS_MASTER_URI']).port
    with socket.socket() as probe:
        try: probe.bind(('127.0.0.1',port))
        except OSError: raise RuntimeError('Exploration ROS Master port is already in use: '+str(port))
    map_dir=WS/'maps'/args.name
    if map_dir.is_symlink() or (WS/'maps').resolve() not in map_dir.resolve().parents:
        raise RuntimeError('Map destination escapes the private maps directory')
    if map_dir.exists() and any(map_dir.iterdir()):
        raise RuntimeError('Refusing to overwrite existing map: '+str(map_dir))
    map_dir.mkdir(parents=True,exist_ok=True)
    (map_dir/'.go2_terrain_mapping_v1').write_text('format: go2_terrain_mapping\nversion: 1\n')
    session_id=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
    directory=WS/'logs'/session_id
    directory.mkdir(parents=True)
    os.environ['ROS_LOG_DIR']=str(directory/'ros')
    auto=args.command=='explore' and not args.no_auto_start
    auto_reenable=auto and not args.no_auto_reenable
    command=['roslaunch','-p',str(port),'go2_exploration','go2_exploration.launch',
             'map_name:='+args.name,'map_root:='+str(WS/'maps'),
             'session_dir:='+str(directory),'auto_start:='+str(auto).lower(),
             'auto_reenable:='+str(auto_reenable).lower(),
             'use_real_sdk:='+str(args.real).lower(),'network_interface:='+args.interface,
             'rviz:='+str(args.rviz).lower(),'camera:='+str(args.camera).lower(),
             'lidar_blind:='+str(args.lidar_blind)]
    child=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
    data={'pid':os.getpid(),'start_ticks':ticks(os.getpid()),'session_id':session_id,
          'launch_pid':child.pid,'session_dir':str(directory),'map_name':args.name,
          'real_sdk':args.real,'auto_start':auto,'master_uri':os.environ['ROS_MASTER_URI'],
          'auto_reenable':auto_reenable,
          'rviz':args.rviz,'camera':args.camera,'display':os.environ.get('DISPLAY',''),
          'lidar_blind_m':args.lidar_blind}
    atomic_json(STATE,data)
    atomic_json(directory/'session.json',data)
    stopping=[False]
    def requested(_signal,_frame): stopping[0]=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP): signal.signal(sig,requested)
    print(('REAL automatic exploration' if args.real and auto else 'MOCK / observation')+'; logs: '+str(directory),flush=True)
    recorder=None
    next_check=time.monotonic()+3
    stop_sent=False
    forced=False
    failure=None
    owned={child.pid:ticks(child.pid)}
    try:
        with (directory/'console.log').open('wb') as output:
            while child.poll() is None:
                readable,_,_=select.select([child.stdout],[],[],.1)
                if readable:
                    chunk=os.read(child.stdout.fileno(),65536)
                    output.write(chunk); output.flush()
                    try: sys.stdout.buffer.write(chunk); sys.stdout.flush()
                    except BrokenPipeError: stopping[0]=True
                now=time.monotonic()
                if now>=next_check:
                    owned.update({pid:ticks(pid) for pid in descendants(child.pid)})
                    other=conflicts(owned)
                    if other:
                        failure='conflicting_robot_processes: '+json.dumps(other)
                        stopping[0]=True
                    next_check=now+1
                if args.record and recorder is None and now>=next_check-1:
                    # Separate record process; never publishes recorded messages.
                    recorder=subprocess.Popen(['rosbag','record','-O',str(directory/'sensors.bag'),
                        '/livox/lidar','/livox/imu','/lio/odometry','/odom_nav','/odom_robot',
                        '/exploration/sensor_ok','/exploration/sensor_status',
                        '/cloud_registered_base','/cloud_registered_odom','/cloud_registered_terrain',
                        '/terrain/ground_points','/terrain/obstacle_points','/terrain/clearing_points',
                        '/exploration/clearing_points','/exploration/clearing_status',
                        '/terrain/healthy','/terrain/status','/tf','/tf_static',
                        '/exploration/odom_ok','/exploration/odom_status','/exploration/map_status',
                        '/nav_static_map','/nav_static_map_metadata',
                        '/exploration/observed_local_map','/exploration/coverage_map','/exploration/coverage_status',
                        '/exploration/frontier_map','/exploration/frontier_status','/exploration/state',
                        '/exploration/recovery_status',
                        '/explore/selected_goal','/explore/selection_status',
                        '/move_base/global_costmap/costmap','/move_base/local_costmap/costmap',
                        '/move_base/GlobalPlanner/plan','/move_base/TebLocalPlannerROS/local_plan',
                        '/move_base/status','/move_base/goal','/cmd_vel_nav','/exploration/cmd_vel_shaped',
                        '/cmd_vel_safe','/go2/control/enabled','/go2/diagnostics','/go2_exploration_safety/status',
                        '/exploration/dashboard','/move_base/TebLocalPlannerROS/execution_mode','/rosout_agg'],
                        stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
                if stopping[0] and not stop_sent:
                    from std_srvs.srv import Trigger
                    try: service('/exploration/stop',Trigger,timeout=2.)
                    except Exception:
                        stop_motion()
                        os.killpg(child.pid,signal.SIGINT)
                        forced=True
                    stop_sent=True
                    stop_deadline=now+90
                if stop_sent and now>stop_deadline:
                    stop_motion()
                    os.killpg(child.pid,signal.SIGINT)
                    forced=True
                    failure=failure or 'shutdown_timeout'
                    break
            if child.poll() is None:
                try: child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    child.wait(timeout=10)
            while select.select([child.stdout],[],[],.1)[0]:
                remaining=os.read(child.stdout.fileno(),65536)
                if not remaining: break
                output.write(remaining)
                try: sys.stdout.buffer.write(remaining);sys.stdout.flush()
                except BrokenPipeError: pass
    finally:
        if child.poll() is None:
            stop_motion()
            os.killpg(child.pid,signal.SIGINT)
            child.wait(timeout=30)
        if recorder and recorder.poll() is None:
            os.killpg(recorder.pid,signal.SIGINT)
            recorder.wait(timeout=30)
        # roslaunch gives nodes separate process groups. Only recorded descendants
        # with the same start time belong to this session; never use killall/pkill.
        for pid,started in owned.items():
            if started is not None and ticks(pid)==started:
                try: os.kill(pid,signal.SIGINT)
                except ProcessLookupError: pass
        result_file=directory/'result.json'
        result=json.loads(result_file.read_text()) if result_file.exists() else {'result':'ABNORMAL_EXIT'}
        saved=(map_dir/'public_map.pcd').exists() and (map_dir/'public_map.pcd').stat().st_size>100
        verified=verify_snapshot(map_dir)
        result.update({'process_exit':child.returncode,'pcd_present':saved,'snapshot_manifest':verified,
                       'forced_shutdown':forced,'failure':failure,
                       'success':not failure and not forced and child.returncode==0 and
                           result.get('save_status')=='saved' and
                           result.get('result') in ('STOPPED','COMPLETED') and saved and verified})
        atomic_json(directory/'session_result.json',result)
        if STATE.exists() and json.loads(STATE.read_text()).get('session_id')==session_id: STATE.unlink()
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return 0 if result['success'] else 1


def main():
    parser=argparse.ArgumentParser(description='Isolated GO2 online exploration')
    sub=parser.add_subparsers(dest='command',required=True)
    for command in ('observe','explore'):
        start=sub.add_parser(command)
        start.add_argument('name')
        start.add_argument('--real',action='store_true')
        start.add_argument('--no-auto-start',action='store_true')
        start.add_argument('--no-auto-reenable',action='store_true',help='Keep recoverable input faults stopped until a new session')
        start.add_argument('--interface',default='eth0')
        viewer=start.add_mutually_exclusive_group()
        viewer.add_argument('--rviz',dest='rviz',action='store_true',help='Open exploration RViz (default)')
        viewer.add_argument('--no-rviz',dest='rviz',action='store_false',help='Run without a graphical viewer')
        start.set_defaults(rviz=True)
        start.add_argument('--no-camera',dest='camera',action='store_false',default=True,
                           help='Disable the D435i video stream')
        start.add_argument('--record',action='store_true')
        start.add_argument('--lidar-blind',type=float,default=.70,
                           help='Per-session Livox near-range rejection in metres (default 0.70)')
    for command in ('status','stop','save-map','rviz'): sub.add_parser(command)
    sub.add_parser('export-map').add_argument('name')
    args=parser.parse_args()
    if args.command in ('observe','explore'): return run(args)
    if args.command=='status': return status()
    if args.command=='rviz': return show_rviz()
    if args.command=='stop': return stop_session()
    if args.command=='save-map':
        if not active_session(): raise RuntimeError('No managed exploration session')
        from std_srvs.srv import Trigger
        result=service('/go2_map_builder/save_map',Trigger,timeout=5.)
        print(result.message)
        return 0 if result.success else 1
    validate_environment()
    return subprocess.call(['/bin/bash',str(WS/'scripts/map_tools.sh'),'export-map',args.name])


if __name__=='__main__':
    try: sys.exit(main())
    except (RuntimeError,OSError,ValueError) as error:
        print('ERROR: '+str(error),file=sys.stderr)
        sys.exit(2)
