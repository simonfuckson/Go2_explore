#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <stdexcept>
#include <vector>

class Go2PoseAdapter
{
public:
  Go2PoseAdapter()
      : nh_(),
        pnh_("~"),
        initialized_(false),
        have_prev_(false),
        init_count_(0),
        sum_x_(0.0),
        sum_y_(0.0),
        sum_z_(0.0),
        qsum_x_(0.0),
        qsum_y_(0.0),
        qsum_z_(0.0),
        qsum_w_(0.0),
        have_q_ref_(false),
        vx_f_(0.0),
        vy_f_(0.0),
        vz_f_(0.0),
        wx_f_(0.0),
        wy_f_(0.0),
        wz_f_(0.0)
  {
    pnh_.param<std::string>("input_odom_topic", input_topic_, "/lio/odometry");
    pnh_.param<std::string>("output_robot_odom_topic", robot_odom_topic_, "/odom_robot");
    pnh_.param<std::string>("output_nav_odom_topic", nav_odom_topic_, "/odom_nav");
    pnh_.param<int>("initialization_samples", init_samples_, 30);
    pnh_.param<double>("velocity_alpha", alpha_, 0.45);

    nh_.param<std::string>("/frames/odom", odom_frame_, "odom");
    nh_.param<std::string>("/frames/base_footprint", footprint_frame_, "base_footprint");
    nh_.param<std::string>("/frames/base_link", base_frame_, "base_link");
    nh_.param<std::string>("/lio_frames/world", lio_world_frame_, "lio_odom");
    nh_.param<std::string>("/lio_frames/body", lio_body_frame_, "body_lio");

    loadMountTransform();

    robot_pub_ = nh_.advertise<nav_msgs::Odometry>(robot_odom_topic_, 20);
    nav_pub_ = nh_.advertise<nav_msgs::Odometry>(nav_odom_topic_, 20);
    sub_ = nh_.subscribe(input_topic_, 100, &Go2PoseAdapter::odomCallback, this);

    ROS_INFO("go2_pose_adapter: input=%s, public odom=%s, base=%s, lio=%s->%s",
             input_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
             lio_world_frame_.c_str(), lio_body_frame_.c_str());
    ROS_INFO("Keep GO2 stationary during the first %d valid odometry samples.",
             init_samples_);
  }

private:
  static double wrap(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double getNumeric(const XmlRpc::XmlRpcValue& cfg, const std::string& key)
  {
    if (!cfg.hasMember(key))
      throw std::runtime_error("Missing calibration key: " + key);

    const XmlRpc::XmlRpcValue& v = cfg[key];
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
      return static_cast<int>(v);
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      return static_cast<double>(v);

    throw std::runtime_error("Calibration key is not numeric: " + key);
  }

  void loadMountTransform()
  {
    XmlRpc::XmlRpcValue cfg;
    if (!nh_.getParam("/mid360_mount/base_link_to_lidar_link", cfg))
      throw std::runtime_error(
          "Missing /mid360_mount/base_link_to_lidar_link. Run V8 migration first.");

    const double x = getNumeric(cfg, "x");
    const double y = getNumeric(cfg, "y");
    const double z = getNumeric(cfg, "z");
    const double roll = getNumeric(cfg, "roll_deg") * M_PI / 180.0;
    const double pitch = getNumeric(cfg, "pitch_deg") * M_PI / 180.0;
    const double yaw = getNumeric(cfg, "yaw_deg") * M_PI / 180.0;

    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    T_base_lidar_.setOrigin(tf::Vector3(x, y, z));
    T_base_lidar_.setRotation(q);

    // FAST-LIO body_lio is the estimator body reference associated with
    // the MID-360 installation calibration. We need body_lio -> base_link.
    T_lidar_base_ = T_base_lidar_.inverse();
  }

  tf::Transform rawBasePose(const nav_msgs::Odometry::ConstPtr& msg) const
  {
    tf::Transform T_lio_body;
    tf::poseMsgToTF(msg->pose.pose, T_lio_body);
    return T_lio_body * T_lidar_base_;
  }

  void accumulateInitialPose(const tf::Transform& T)
  {
    const tf::Vector3 p = T.getOrigin();
    tf::Quaternion q = T.getRotation();
    q.normalize();

    if (!have_q_ref_)
    {
      q_ref_ = q;
      have_q_ref_ = true;
    }
    else if (q_ref_.dot(q) < 0.0)
    {
      q = tf::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
    }

    sum_x_ += p.x();
    sum_y_ += p.y();
    sum_z_ += p.z();
    qsum_x_ += q.x();
    qsum_y_ += q.y();
    qsum_z_ += q.z();
    qsum_w_ += q.w();
    ++init_count_;
  }

  void finishInitialization(const ros::Time& stamp)
  {
    const double n = static_cast<double>(init_count_);

    tf::Quaternion q(qsum_x_ / n, qsum_y_ / n, qsum_z_ / n, qsum_w_ / n);
    q.normalize();

    T_lio_base0_.setOrigin(tf::Vector3(sum_x_ / n, sum_y_ / n, sum_z_ / n));
    T_lio_base0_.setRotation(q);

    // Public odom is defined by the startup GO2 base_link pose:
    // T_odom_lio * T_lio_base(t0) = I
    T_odom_lio_ = T_lio_base0_.inverse();

    geometry_msgs::TransformStamped st;
    st.header.stamp = stamp;
    st.header.frame_id = odom_frame_;
    st.child_frame_id = lio_world_frame_;

    const tf::Vector3 t = T_odom_lio_.getOrigin();
    const tf::Quaternion qs = T_odom_lio_.getRotation();

    st.transform.translation.x = t.x();
    st.transform.translation.y = t.y();
    st.transform.translation.z = t.z();
    st.transform.rotation.x = qs.x();
    st.transform.rotation.y = qs.y();
    st.transform.rotation.z = qs.z();
    st.transform.rotation.w = qs.w();

    static_broadcaster_.sendTransform(st);

    nh_.setParam("/go2_pose_adapter/initialized", true);
    nh_.setParam("/go2_pose_adapter/init_samples_used", init_count_);

    initialized_ = true;

    ROS_INFO("go2_pose_adapter initialized with %d samples.", init_count_);
    ROS_INFO("Published static alignment TF: %s -> %s",
             odom_frame_.c_str(), lio_world_frame_.c_str());
    ROS_INFO("At initialization: odom == robot_init == base_footprint == base_link.");
  }

  void publishDynamicTf(const tf::Transform& T_odom_base, const ros::Time& stamp)
  {
    const tf::Vector3 p = T_odom_base.getOrigin();

    double roll, pitch, yaw;
    tf::Matrix3x3(T_odom_base.getRotation()).getRPY(roll, pitch, yaw);

    tf::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, yaw);

    tf::Transform T_odom_footprint;
    T_odom_footprint.setOrigin(tf::Vector3(p.x(), p.y(), 0.0));
    T_odom_footprint.setRotation(q_yaw);

    const tf::Transform T_footprint_base =
        T_odom_footprint.inverse() * T_odom_base;

    dynamic_broadcaster_.sendTransform(
        tf::StampedTransform(T_odom_footprint, stamp,
                             odom_frame_, footprint_frame_));

    dynamic_broadcaster_.sendTransform(
        tf::StampedTransform(T_footprint_base, stamp,
                             footprint_frame_, base_frame_));
  }

  void fillTwist(const tf::Transform& T_odom_base,
                 const ros::Time& stamp,
                 nav_msgs::Odometry& robot,
                 nav_msgs::Odometry& nav)
  {
    const tf::Vector3 p = T_odom_base.getOrigin();
    double r, pt, y;
    tf::Matrix3x3(T_odom_base.getRotation()).getRPY(r, pt, y);

    if (have_prev_)
    {
      const double dt = (stamp - prev_stamp_).toSec();
      if (dt > 0.002 && dt < 1.0)
      {
        const double vx_w = (p.x() - prev_x_) / dt;
        const double vy_w = (p.y() - prev_y_) / dt;
        const double vz_w = (p.z() - prev_z_) / dt;

        const double c = std::cos(y);
        const double s = std::sin(y);

        const double vx_b = c * vx_w + s * vy_w;
        const double vy_b = -s * vx_w + c * vy_w;

        const double wx = wrap(r - prev_roll_) / dt;
        const double wy = wrap(pt - prev_pitch_) / dt;
        const double wz = wrap(y - prev_yaw_) / dt;

        vx_f_ = alpha_ * vx_b + (1.0 - alpha_) * vx_f_;
        vy_f_ = alpha_ * vy_b + (1.0 - alpha_) * vy_f_;
        vz_f_ = alpha_ * vz_w + (1.0 - alpha_) * vz_f_;
        wx_f_ = alpha_ * wx + (1.0 - alpha_) * wx_f_;
        wy_f_ = alpha_ * wy + (1.0 - alpha_) * wy_f_;
        wz_f_ = alpha_ * wz + (1.0 - alpha_) * wz_f_;

        robot.twist.twist.linear.x = vx_f_;
        robot.twist.twist.linear.y = vy_f_;
        robot.twist.twist.linear.z = vz_f_;
        robot.twist.twist.angular.x = wx_f_;
        robot.twist.twist.angular.y = wy_f_;
        robot.twist.twist.angular.z = wz_f_;

        nav.twist.twist.linear.x = vx_f_;
        nav.twist.twist.linear.y = vy_f_;
        nav.twist.twist.angular.z = wz_f_;
      }
    }

    prev_stamp_ = stamp;
    prev_x_ = p.x();
    prev_y_ = p.y();
    prev_z_ = p.z();
    prev_roll_ = r;
    prev_pitch_ = pt;
    prev_yaw_ = y;
    have_prev_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if (!msg->header.frame_id.empty() &&
        msg->header.frame_id != lio_world_frame_)
    {
      ROS_WARN_THROTTLE(3.0, "Expected odometry frame_id=%s, got %s",
                        lio_world_frame_.c_str(), msg->header.frame_id.c_str());
    }

    if (!msg->child_frame_id.empty() &&
        msg->child_frame_id != lio_body_frame_)
    {
      ROS_WARN_THROTTLE(3.0, "Expected odometry child_frame_id=%s, got %s",
                        lio_body_frame_.c_str(), msg->child_frame_id.c_str());
    }

    const tf::Transform T_lio_base = rawBasePose(msg);

    if (!initialized_)
    {
      accumulateInitialPose(T_lio_base);
      if (init_count_ >= std::max(1, init_samples_))
        finishInitialization(msg->header.stamp);
      else
        ROS_INFO_THROTTLE(1.0, "Initializing GO2 public odom: %d/%d",
                          init_count_, init_samples_);
      return;
    }

    const tf::Transform T_odom_base = T_odom_lio_ * T_lio_base;
    publishDynamicTf(T_odom_base, msg->header.stamp);

    nav_msgs::Odometry robot;
    robot.header.stamp = msg->header.stamp;
    robot.header.frame_id = odom_frame_;
    robot.child_frame_id = base_frame_;
    tf::poseTFToMsg(T_odom_base, robot.pose.pose);

    const tf::Vector3 p = T_odom_base.getOrigin();
    double roll, pitch, yaw;
    tf::Matrix3x3(T_odom_base.getRotation()).getRPY(roll, pitch, yaw);

    nav_msgs::Odometry nav;
    nav.header.stamp = msg->header.stamp;
    nav.header.frame_id = odom_frame_;
    nav.child_frame_id = footprint_frame_;
    nav.pose.pose.position.x = p.x();
    nav.pose.pose.position.y = p.y();
    nav.pose.pose.position.z = 0.0;
    tf::Quaternion q_nav;
    q_nav.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(q_nav, nav.pose.pose.orientation);

    fillTwist(T_odom_base, msg->header.stamp, robot, nav);

    robot_pub_.publish(robot);
    nav_pub_.publish(nav);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher robot_pub_;
  ros::Publisher nav_pub_;

  tf::TransformBroadcaster dynamic_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  std::string input_topic_;
  std::string robot_odom_topic_;
  std::string nav_odom_topic_;
  std::string odom_frame_;
  std::string footprint_frame_;
  std::string base_frame_;
  std::string lio_world_frame_;
  std::string lio_body_frame_;

  tf::Transform T_base_lidar_;
  tf::Transform T_lidar_base_;
  tf::Transform T_lio_base0_;
  tf::Transform T_odom_lio_;

  bool initialized_;
  bool have_prev_;
  int init_samples_;
  int init_count_;
  double alpha_;

  double sum_x_, sum_y_, sum_z_;
  double qsum_x_, qsum_y_, qsum_z_, qsum_w_;
  bool have_q_ref_;
  tf::Quaternion q_ref_;

  ros::Time prev_stamp_;
  double prev_x_, prev_y_, prev_z_;
  double prev_roll_, prev_pitch_, prev_yaw_;
  double vx_f_, vy_f_, vz_f_, wx_f_, wy_f_, wz_f_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go2_pose_adapter");
  try
  {
    Go2PoseAdapter node;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("go2_pose_adapter fatal: %s", e.what());
    return 2;
  }
  return 0;
}
