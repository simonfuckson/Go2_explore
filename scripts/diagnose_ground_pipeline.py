#!/usr/bin/env python3
"""Validate ground/coverage on private diagnostic topics using current sensors.

Does not publish to the active exploration/control inputs, enable the SDK,
send goals, start hardware drivers or modify active node parameters.
"""
from collections import Counter,defaultdict
import copy,json,os,signal,subprocess,time,uuid
from pathlib import Path
import xml.etree.ElementTree as ET
import rospy,rosgraph
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool,String

ROOT=Path(__file__).resolve().parents[1]

def main():
    rospy.init_node('ground_pipeline_diagnostic',anonymous=True,disable_signals=True)
    directory=ROOT/'artifacts'/('ground_pipeline_'+uuid.uuid4().hex[:8]);directory.mkdir()
    prefix='/exploration_diagnostics_'+uuid.uuid4().hex[:8]
    counts=Counter();statuses=defaultdict(Counter);samples=defaultdict(list);last={};control=[]
    def callback(message,topic):
        counts[topic]+=1
        if hasattr(message,'header'):
            samples[topic].append((time.monotonic(),(rospy.Time.now()-message.header.stamp).to_sec()))
        if isinstance(message,PointCloud2):last[topic]={'points':message.width*message.height,'frame':message.header.frame_id}
        elif isinstance(message,OccupancyGrid):last[topic]={'known_cells':sum(v>=0 for v in message.data),'width':message.info.width,'height':message.info.height}
        elif isinstance(message,String):statuses[topic][message.data.split(':')[0]]+=1;last[topic]=message.data
        elif isinstance(message,Bool):statuses[topic][str(message.data)]+=1
    node_names={kind:prefix.strip('/')+'_'+kind for kind in ('guard','memory','frontiers')}
    graph=ET.Element('launch')
    guard=ET.SubElement(graph,'node',pkg='go2_terrain',type='go2_terrain_guard_node',name=node_names['guard'],output='screen')
    params=copy.deepcopy(rospy.get_param('/go2_terrain_guard'))
    params.setdefault('output',{})['publish_debug_clouds']=True
    rospy.set_param('/'+node_names['guard'],params)
    for source,target in [('ground','/terrain/patchwork_ground'),('nonground','/terrain/patchwork_nonground'),
                          ('safe_ground',prefix+'/ground'),('obstacles',prefix+'/obstacles'),('clearing',prefix+'/clearing'),
                          ('unknown',prefix+'/unknown'),('diagnostics',prefix+'/guard_status'),('healthy',prefix+'/healthy')]:
        ET.SubElement(guard,'remap',{'from':source,'to':target})
    memory=ET.SubElement(graph,'node',pkg='go2_exploration',type='observed_map_memory.py',name=node_names['memory'],output='screen')
    for source,target in [('/terrain/ground_points',prefix+'/ground'),('/nav_static_map',prefix+'/map'),
                          ('/nav_static_map_metadata',prefix+'/map_metadata'),('/exploration/map_status',prefix+'/map_status'),
                          ('/exploration/coverage_map',prefix+'/coverage'),('/exploration/coverage_age',prefix+'/coverage_age')]:
        ET.SubElement(memory,'remap',{'from':source,'to':target})
    frontier=ET.SubElement(graph,'node',pkg='go2_exploration',type='frontier_costmap_adapter.py',name=node_names['frontiers'],output='screen')
    for name,value in [('input_topic',prefix+'/map'),('output_topic',prefix+'/frontier_map'),('updates_topic',prefix+'/unused_updates')]:
        ET.SubElement(frontier,'param',name=name,value=value)
    for source,target in [('/exploration/coverage_map',prefix+'/coverage'),('/exploration/coverage_status',prefix+'/coverage_status'),
                          ('/exploration/frontier_status',prefix+'/frontier_status'),('/exploration/frontier_available',prefix+'/frontier_available')]:
        ET.SubElement(frontier,'remap',{'from':source,'to':target})
    launch=directory/'passive.launch';ET.ElementTree(graph).write(str(launch))
    topics={'ground':PointCloud2,'healthy':Bool,'map':OccupancyGrid,'coverage':OccupancyGrid,
            'map_status':String,'coverage_status':String,'frontier_status':String}
    subscriptions=[rospy.Subscriber(prefix+'/'+topic,kind,callback,callback_args=topic,queue_size=5) for topic,kind in topics.items()]
    subscriptions.append(rospy.Subscriber('/go2/control/enabled',Bool,lambda m:control.append(m.data),queue_size=5))
    environment=dict(os.environ,ROS_LOG_DIR=str(directory/'ros'))
    log=(directory/'console.log').open('wb')
    child=subprocess.Popen(['roslaunch',str(launch)],stdout=log,stderr=subprocess.STDOUT,env=environment,start_new_session=True)
    result={'directory':str(directory),'prefix':prefix,'uses_real_sensors':True,'sends_control_or_goals':False}
    try:
        start=time.monotonic()
        while time.monotonic()-start<45:
            if child.poll() is not None:raise RuntimeError('Diagnostic launch exited early')
            if rospy.is_shutdown():raise RuntimeError('Diagnostic observer shut down')
            time.sleep(.2)
        publishers=rosgraph.Master(rospy.get_name()).getSystemState()[0]
        own={'/'+name for name in node_names.values()}
        outputs={topic:nodes for topic,nodes in publishers if own.intersection(nodes)}
        assert all(topic.startswith(prefix+'/') or topic=='/rosout' for topic in outputs),outputs
        assert rospy.get_param('/go2_terrain_guard/output/publish_debug_clouds') is False
        assert not any(control),'Active controller enabled during observation'
        assert counts['ground']>=100 and statuses['map_status']['ready']>=10,(dict(counts),dict(statuses))
        assert last['coverage']['known_cells']>0 and statuses['coverage_status']['ready']>=10
        result.update(passed=True,publication_isolated=True,active_ground_switch_unchanged=True,active_control_enabled=False,
                      duration_seconds=time.monotonic()-start,outputs=outputs)
    except Exception as error:result.update(passed=False,error=repr(error))
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        log.close()
        for name in node_names.values():
            if rospy.has_param('/'+name):rospy.delete_param('/'+name)
        result.update(counts=dict(counts),statuses={t:dict(v) for t,v in statuses.items()},last=last,
                      median_ages={t:sorted(v[1] for v in data)[len(data)//2] for t,data in samples.items() if data})
        (directory/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
