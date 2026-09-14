#!/usr/bin/env python3
"""Rasterize the GO2 guard's measured support cells at the costmap spacing."""
import math
import numpy as np

def expand_support_cells(points,grid_resolution,grid_origin,map_resolution,min_range,max_range):
    points=np.asarray(points,dtype=float).reshape(-1,3)
    if not all(math.isfinite(v) and v>0 for v in (grid_resolution,map_resolution,max_range)) or min_range<0:
        raise ValueError('invalid support grid geometry')
    points=points[np.isfinite(points).all(axis=1)]
    if not len(points):return np.empty((0,3))
    origin=np.asarray(grid_origin,dtype=float)
    keys=np.floor((points[:,:2]-origin)/grid_resolution).astype(np.int64)
    centers=origin+(keys+.5)*grid_resolution
    # This input is a grid of confirmed ground representatives. Reject a raw
    # point cloud accidentally wired here instead of expanding its endpoints.
    if np.any(np.abs(centers-points[:,:2])>1e-4):raise ValueError('clearing input is not GO2 support-cell centers')
    _,indices=np.unique(keys,axis=0,return_index=True);keys=keys[indices];points=points[indices]
    step=map_resolution*.5
    xx,yy=np.meshgrid(np.arange(step*.5,grid_resolution,step),np.arange(step*.5,grid_resolution,step))
    offsets=np.column_stack((xx.ravel(),yy.ravel()))
    xy=(origin+keys[:,None,:]*grid_resolution+offsets).reshape(-1,2)
    output=np.column_stack((xy,np.repeat(points[:,2],len(offsets))))
    r=np.linalg.norm(output[:,:2],axis=1)
    return output[(r>=min_range)&(r<=max_range)]

def main():
    import rospy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.point_cloud2 import create_cloud_xyz32,read_points
    from std_msgs.msg import String
    rospy.init_node('go2_exploration_ground_clearing')
    grid=rospy.get_param('/go2_terrain_guard/grid');ranges=rospy.get_param('/go2_terrain_guard/vehicle')
    resolution=rospy.get_param('/move_base/local_costmap/resolution')
    pub=rospy.Publisher('/exploration/clearing_points',PointCloud2,queue_size=1)
    status=rospy.Publisher('/exploration/clearing_status',String,queue_size=1,latch=True)
    def receive(cloud):
        try:
            if cloud.header.frame_id!='terrain_sensor' or not -.1<=(rospy.Time.now()-cloud.header.stamp).to_sec()<=.6:
                raise ValueError('support frame or timestamp invalid')
            points=expand_support_cells(list(read_points(cloud,field_names=('x','y','z'),skip_nans=True)),grid['resolution'],(grid['min_x'],grid['min_y']),resolution,ranges['min_horizontal_range'],ranges['max_horizontal_range'])
            pub.publish(create_cloud_xyz32(cloud.header,points.tolist()))
            status.publish(String(data='ready: support_cells=%d clearing_samples=%d'%(cloud.width*cloud.height,len(points))))
        except (ValueError,KeyError,TypeError) as error:
            status.publish(String(data='invalid: '+str(error)));rospy.logerr_throttle(2.,'Exploration clearing: %s',error)
    rospy.Subscriber('/terrain/clearing_points',PointCloud2,receive,queue_size=1)
    rospy.spin()

if __name__=='__main__':main()
