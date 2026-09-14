#!/usr/bin/env python3
"""Summarize a recorded static acceptance session without replaying commands."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import rosbag

def main():
    parser=argparse.ArgumentParser();parser.add_argument('bag');args=parser.parse_args()
    bag_path=Path(args.bag)
    report={'bag':str(bag_path),'topics':{},'states':{},'nonzero_final_commands':0,'control_enabled_true':0}
    string_counts=defaultdict(Counter);counts=Counter();stamps=defaultdict(set);sizes=defaultdict(list);callers=defaultdict(set)
    interesting=['/exploration/state','/exploration/odom_status','/exploration/map_status',
        '/exploration/coverage_status','/exploration/frontier_status','/explore/selection_status',
        '/terrain/healthy','/exploration/odom_ok','/go2/control/enabled','/cmd_vel_safe',
        '/explore/selected_goal','/move_base/GlobalPlanner/plan','/move_base/TebLocalPlannerROS/local_plan']
    with rosbag.Bag(str(bag_path)) as bag:
        for connection in bag._get_connections(topics=['/cmd_vel_safe']):
            caller=connection.header.get('callerid','unknown')
            callers['/cmd_vel_safe'].add(caller.decode() if isinstance(caller,bytes) else caller)
        report['duration_seconds']=bag.get_end_time()-bag.get_start_time()
        info=bag.get_type_and_topic_info()[1]
        report['topics']={k:{'messages':v.message_count,'type':v.msg_type,'frequency':v.frequency} for k,v in info.items()}
        for topic,msg,t in bag.read_messages(topics=interesting):
            counts[topic]+=1
            if hasattr(msg,'data') and isinstance(msg.data,(str,bool)):
                value=msg.data
                if topic in ('/exploration/map_status','/exploration/coverage_status'):
                    if value.startswith('ready:'): stamps[topic].add(value)
                    value=value.split(':')[0]
                string_counts[topic][str(value)]+=1
            if topic=='/cmd_vel_safe':
                if abs(msg.linear.x)+abs(msg.linear.y)+abs(msg.angular.z)>1e-8:report['nonzero_final_commands']+=1
            if topic=='/go2/control/enabled' and msg.data:report['control_enabled_true']+=1
            if hasattr(msg,'poses'):sizes[topic].append(len(msg.poses))
        report['states']={t:dict(v) for t,v in string_counts.items()}
        report['publishers']={t:sorted(v) for t,v in callers.items()}
        report['unique_map_updates']={t:len(v) for t,v in stamps.items()}
        report['plans']={t:{'messages':len(v),'nonempty':sum(x>0 for x in v),'max_poses':max(v)} for t,v in sizes.items()}
    report['stationary_acceptance']=(report['duration_seconds']>=180 and report['nonzero_final_commands']==0 and
        report['control_enabled_true']==0 and report['unique_map_updates'].get('/exploration/map_status',0)>=2 and
        counts['/explore/selected_goal']>0 and string_counts['/terrain/healthy']['True']>0 and
        string_counts['/exploration/odom_ok']['True']>0 and any(v['nonempty']>0 for v in report['plans'].values()))
    output=bag_path.parent/'static_bag_report.json';output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__': main()
