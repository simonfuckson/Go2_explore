#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

class MockSdkBridge
{
public:
  MockSdkBridge()
      : nh_(),
        pnh_("~"),
        enabled_(false)
  {
    command_sub_ = nh_.subscribe(
        "/cmd_vel_safe",
        10,
        &MockSdkBridge::commandCallback,
        this);

    enable_service_ = pnh_.advertiseService(
        "enable",
        &MockSdkBridge::enableCallback,
        this);
    control_enabled_pub_ = nh_.advertise<std_msgs::Bool>(
        "/go2/control/enabled", 1, true);
    publishControlEnabled();

    ROS_WARN(
        "MOCK SDK bridge active: "
        "NO command is sent to GO2.");
  }

private:
  void commandCallback(
      const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!enabled_)
    {
      return;
    }

    ROS_INFO_THROTTLE(
        0.5,
        "MOCK GO2 cmd: vx=%.3f vy=%.3f wz=%.3f",
        msg->linear.x,
        msg->linear.y,
        msg->angular.z);
  }

  bool enableCallback(
      std_srvs::SetBool::Request& request,
      std_srvs::SetBool::Response& response)
  {
    enabled_ = request.data;
    publishControlEnabled();

    response.success = true;
    response.message =
        enabled_
            ? "mock bridge enabled"
            : "mock bridge disabled";

    return true;
  }

  void publishControlEnabled()
  {
    std_msgs::Bool msg;
    msg.data = enabled_;
    control_enabled_pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber command_sub_;
  ros::ServiceServer enable_service_;
  ros::Publisher control_enabled_pub_;

  bool enabled_;
};

int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_sdk_bridge_mock");

  MockSdkBridge node;

  ros::spin();

  return 0;
}
