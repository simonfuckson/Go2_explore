#!/usr/bin/env python3
"""Bounded passive ROS evidence, including motion after manual takeover."""
import json
from pathlib import Path
import queue
import shutil
import threading
import time


class EvidenceBudget:
    def __init__(self, byte_limit=128*1024*1024, minimum_free=512*1024*1024):
        if byte_limit<=0 or minimum_free<0:raise ValueError('invalid evidence budget')
        self.byte_limit,self.minimum_free=byte_limit,minimum_free

    def problem(self, size, free):
        if size>=self.byte_limit:return 'size_limit_reached'
        if free<self.minimum_free:return 'disk_space_low'
        return None


def main():
    import rospy,rosbag
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String,Bool
    from tf2_msgs.msg import TFMessage
    rospy.init_node('go2_motion_evidence')
    folder=Path(rospy.get_param('~session_dir')).resolve()
    ws=(Path.home()/'go2_explore_ws').resolve()
    if ws not in folder.parents:raise ValueError('Evidence must stay inside explore workspace')
    folder.mkdir(parents=True,exist_ok=True)
    path=folder/'motion_evidence.bag'
    if path.exists():raise ValueError('Refusing to overwrite existing motion evidence')
    budget=EvidenceBudget(int(rospy.get_param('~max_bytes',128*1024*1024)))
    work=queue.Queue(maxsize=256);finished=threading.Event();lock=threading.Lock()
    counts={};dropped=[0];last={};reason=['recording'];started=time.monotonic()
    duration=float(rospy.get_param('~duration',0))
    status=rospy.Publisher('/exploration/evidence_status',String,queue_size=1,latch=True)
    def receive(message,topic):
        now=time.monotonic()
        with lock:
            if finished.is_set():return
            # Keep all odometry samples and TF; point clouds are full frames at
            # 2 Hz, diagnostics at 2 Hz. Original sensor timestamps are retained.
            period=.5 if isinstance(message,(PointCloud2,DiagnosticArray)) else 0
            if now-last.get(topic,-1e10)<period:return
            last[topic]=now
            try:work.put_nowait((topic,message,rospy.Time.now()))
            except queue.Full:dropped[0]+=1
    def writer():
        bag=None
        try:
            problem=budget.problem(0,shutil.disk_usage(folder).free)
            if problem:reason[0]=problem;return
            bag=rosbag.Bag(str(path),'w',compression='lz4',chunk_threshold=256*1024)
            flushed=time.monotonic()
            while not finished.is_set() or not work.empty():
                try:topic,message,stamp=work.get(timeout=.1)
                except queue.Empty:continue
                problem=budget.problem(bag.size,shutil.disk_usage(folder).free)
                if problem:reason[0]=problem;break
                bag.write(topic,message,t=stamp,connection_header=getattr(message,'_connection_header',None))
                with lock:counts[topic]=counts.get(topic,0)+1
                if time.monotonic()-flushed>2:bag.flush();flushed=time.monotonic()
        except Exception as error:
            reason[0]='write_failed: '+str(error);rospy.logerr('Motion evidence: %s',error)
        finally:
            finished.set()
            if bag is not None:
                try:bag.close()
                except Exception as error:reason[0]='close_failed: '+str(error)
            if reason[0]=='recording':reason[0]='closed'
            report=dict(reason=reason[0],topics=counts,dropped_messages=dropped[0],
                elapsed_s=time.monotonic()-started,cloud_hz_limit=2,bytes=path.stat().st_size if path.exists() else 0)
            (folder/'motion_evidence.json').write_text(json.dumps(report,indent=2))
    specs={t:Odometry for t in ('/lio/odometry','/odom_nav','/odom_robot')}
    specs.update({t:DiagnosticArray for t in ('/go2/diagnostics','/go2_exploration_safety/status','/terrain/status')})
    specs.update({t:PointCloud2 for t in ('/cloud_registered_terrain','/terrain/ground_points','/terrain/obstacle_points')})
    specs.update({t:Twist for t in ('/cmd_vel_nav','/exploration/cmd_vel_shaped','/cmd_vel_safe')})
    specs.update({t:String for t in ('/exploration/state','/exploration/odom_status','/explore/selection_status')})
    specs.update({'/go2/control/enabled':Bool,'/tf':TFMessage,'/tf_static':TFMessage})
    subs=[rospy.Subscriber(t,k,receive,callback_args=t,queue_size=100) for t,k in specs.items()]
    thread=threading.Thread(target=writer,daemon=True);thread.start()
    try:
        while not rospy.is_shutdown() and not finished.is_set():
            if duration>0 and time.monotonic()-started>=duration:break
            with lock:message_count=sum(counts.values())
            status.publish(String(data='recording: messages=%d dropped=%d bytes=%d'%(
                message_count,dropped[0],path.stat().st_size if path.exists() else 0)))
            time.sleep(.5)
    finally:
        for sub in subs:sub.unregister()
        finished.set();thread.join(timeout=10)
        status.publish(String(data=reason[0]))


if __name__=='__main__':main()
