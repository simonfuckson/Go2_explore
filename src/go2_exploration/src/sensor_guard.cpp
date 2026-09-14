#include "sensor_continuity.h"
#include "camera_self_filter.h"
#include <memory>
#include <algorithm>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

class SensorGuard {
  ros::NodeHandle nh_, private_{"~"};
  ros::Publisher lidar_, imu_, health_, status_;
  ros::Subscriber lidar_in_, imu_in_;
  ros::WallTimer timer_;
  go2_exploration::SensorContinuity state_;
  std::unique_ptr<go2_exploration::CameraSelfFilter> self_filter_;
  std::string interface_, previous_status_;
  long changes_=-1;
  double stable_since_=0;
  long read(const std::string& entry) {
    long value=-1;
    std::ifstream file("/sys/class/net/"+interface_+"/"+entry);file>>value;return value;
  }
  void lidar(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
    unsigned maximum=0;
    for (const auto& point:msg->points) maximum=std::max(maximum,point.offset_time);
    if (msg->point_num!=msg->points.size()) {state_.fail("lidar_point_count_mismatch");return;}
    if (state_.accept(true,msg->header.stamp.toSec(),ros::Time::now().toSec(),ros::WallTime::now().toSec(),msg->point_num,maximum*1e-9)) {
      const auto filtered=self_filter_->filter(msg);
      if (filtered) lidar_.publish(filtered);
    }
  }
  void imu(const sensor_msgs::Imu::ConstPtr& msg) {
    for (double v:{msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z,
                   msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z})
      if (!std::isfinite(v)) {state_.fail("imu_nonfinite_data");return;}
    if (state_.accept(false,msg->header.stamp.toSec(),ros::Time::now().toSec(),ros::WallTime::now().toSec())) imu_.publish(msg);
  }
  void tick(const ros::WallTimerEvent&) {
    const double now=ros::WallTime::now().toSec();
    const long carrier=read("carrier"), changes=read("carrier_changes");
    if (state_.lidar_stamp && (carrier!=1 || (changes_>=0 && changes!=changes_)))
      state_.fail("lidar_network_link_lost:"+interface_);
    if (!state_.lidar_stamp) changes_=changes;
    state_.watchdog(now);
    const bool inputs=state_.fault.empty() && carrier==1 && state_.lidar_stamp && state_.imu_stamp;
    if (!inputs) stable_since_=0; else if (!stable_since_) stable_since_=now;
    const bool healthy=inputs && now-stable_since_>=1.5;
    std_msgs::Bool ok;ok.data=healthy;health_.publish(ok);
    std_msgs::String status;
    status.data=!state_.fault.empty() ? "FAULT: "+state_.fault+"; stopped feeding FAST-LIO; restart session after repairing sensor link" :
       healthy ? "healthy: "+interface_+" link and lidar/IMU timing stable" : "waiting: "+interface_+" link and continuous lidar/IMU";
    if (status.data!=previous_status_) {
      if (!state_.fault.empty()) ROS_ERROR_STREAM(status.data); else ROS_INFO_STREAM(status.data);
      previous_status_=status.data;
    }
    status_.publish(status);
    self_filter_->publishStatus();
  }
 public:
  SensorGuard() {
    private_.param<std::string>("lidar_interface",interface_,"eth1");
    self_filter_.reset(new go2_exploration::CameraSelfFilter([this](const std::string& reason){state_.fail(reason);}));
    lidar_=nh_.advertise<livox_ros_driver2::CustomMsg>("/exploration/livox/lidar_validated",5);
    imu_=nh_.advertise<sensor_msgs::Imu>("/exploration/livox/imu_validated",100);
    health_=nh_.advertise<std_msgs::Bool>("/exploration/sensor_ok",1,true);
    status_=nh_.advertise<std_msgs::String>("/exploration/sensor_status",1,true);
    lidar_in_=nh_.subscribe("/livox/lidar",5,&SensorGuard::lidar,this);
    imu_in_=nh_.subscribe("/livox/imu",100,&SensorGuard::imu,this);
    changes_=read("carrier_changes");
    timer_=nh_.createWallTimer(ros::WallDuration(.025),&SensorGuard::tick,this);
  }
};
int main(int argc,char** argv) {ros::init(argc,argv,"go2_exploration_sensor_guard");SensorGuard node;ros::spin();}
