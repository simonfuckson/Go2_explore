#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <string>

class LocalizationGuard
{
public:
  LocalizationGuard()
      : nh_(),
        pnh_("~"),
        have_prev_(false),
        upstream_ok_(false),
        localization_ok_(false),
        good_count_(0),
        bad_count_(0)
  {
    pnh_.param<std::string>(
        "global_pose_topic",
        global_pose_topic_,
        "/localization");

    pnh_.param<std::string>(
        "upstream_health_topic",
        upstream_health_topic_,
        "/localization/ndt_ok");

    pnh_.param<std::string>(
        "map_frame",
        map_frame_,
        "map");

    pnh_.param<double>(
        "timeout_sec",
        timeout_sec_,
        2.0);

    pnh_.param<double>(
        "max_jump_m",
        max_jump_m_,
        0.8);

    pnh_.param<double>(
        "max_jump_yaw_deg",
        max_jump_yaw_deg_,
        20.0);

    pnh_.param<int>(
        "good_required_count",
        good_required_count_,
        5);

    pnh_.param<int>(
        "lost_required_count",
        lost_required_count_,
        3);

    pose_sub_ = nh_.subscribe(
        global_pose_topic_,
        10,
        &LocalizationGuard::poseCallback,
        this);

    upstream_health_sub_ = nh_.subscribe(
        upstream_health_topic_, 10,
        &LocalizationGuard::upstreamHealthCallback, this);

    state_pub_ = nh_.advertise<std_msgs::String>(
        "/localization/state",
        10,
        true);

    ok_pub_ = nh_.advertise<std_msgs::Bool>(
        "/localization/ok",
        10,
        true);

    confidence_pub_ =
        nh_.advertise<std_msgs::Float32>(
            "/localization/confidence",
            10,
            true);

    timer_ = nh_.createTimer(
        ros::Duration(0.2),
        &LocalizationGuard::timerCallback,
        this);

    publishState(
        "INIT",
        false,
        0.0f);
  }

private:
  static double wrapAngle(double a)
  {
    while (a > M_PI)
    {
      a -= 2.0 * M_PI;
    }

    while (a < -M_PI)
    {
      a += 2.0 * M_PI;
    }

    return a;
  }

  static double yawFromQuaternion(
      const geometry_msgs::Quaternion& qmsg)
  {
    tf2::Quaternion q(
        qmsg.x,
        qmsg.y,
        qmsg.z,
        qmsg.w);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    tf2::Matrix3x3(q).getRPY(
        roll,
        pitch,
        yaw);

    return yaw;
  }

  void publishState(
      const std::string& state,
      bool ok,
      float confidence)
  {
    std_msgs::String state_msg;
    state_msg.data = state;
    state_pub_.publish(state_msg);

    std_msgs::Bool ok_msg;
    ok_msg.data = ok;
    ok_pub_.publish(ok_msg);

    std_msgs::Float32 confidence_msg;
    confidence_msg.data = confidence;
    confidence_pub_.publish(confidence_msg);
  }

  void poseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    bool sample_good = upstream_ok_ && msg->header.frame_id == map_frame_;

    const auto& p = msg->pose.position;
    const auto& q = msg->pose.orientation;
    sample_good = sample_good &&
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(q.x) && std::isfinite(q.y) &&
        std::isfinite(q.z) && std::isfinite(q.w);

    if (have_prev_)
    {
      const double dx =
          msg->pose.position.x -
          previous_pose_.pose.position.x;

      const double dy =
          msg->pose.position.y -
          previous_pose_.pose.position.y;

      const double dz =
          msg->pose.position.z -
          previous_pose_.pose.position.z;

      const double distance =
          std::sqrt(
              dx * dx +
              dy * dy +
              dz * dz);

      const double yaw_now =
          yawFromQuaternion(
              msg->pose.orientation);

      const double yaw_previous =
          yawFromQuaternion(
              previous_pose_.pose.orientation);

      const double yaw_jump_deg =
          std::fabs(
              wrapAngle(
                  yaw_now -
                  yaw_previous)) *
          180.0 / M_PI;

      if (distance > max_jump_m_)
      {
        sample_good = false;

        ROS_WARN(
            "Localization jump %.3f m > %.3f m",
            distance,
            max_jump_m_);
      }

      if (yaw_jump_deg > max_jump_yaw_deg_)
      {
        sample_good = false;

        ROS_WARN(
            "Localization yaw jump %.2f deg > %.2f deg",
            yaw_jump_deg,
            max_jump_yaw_deg_);
      }
    }

    previous_pose_ = *msg;
    last_pose_wall_time_ = ros::WallTime::now();
    have_prev_ = true;

    if (sample_good)
    {
      good_count_++;
      bad_count_ = 0;

      if (localization_ok_ || good_count_ >= good_required_count_)
      {
        localization_ok_ = true;
        publishState(
            "GOOD",
            true,
            1.0f);
      }
      else
      {
        publishState(
            "INIT",
            false,
            0.5f);
      }
    }
    else
    {
      good_count_ = 0;
      bad_count_++;

      if (localization_ok_ && bad_count_ < lost_required_count_)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "Localization sample degraded (%d/%d); retaining last GOOD state",
            bad_count_,
            lost_required_count_);
        publishState(
            "DEGRADED",
            true,
            0.5f);
      }
      else if (bad_count_ >= lost_required_count_)
      {
        localization_ok_ = false;
        publishState(
            "LOST",
            false,
            0.0f);
      }
      else
      {
        publishState(
            "INIT",
            false,
            0.5f);
      }
    }
  }

  void timerCallback(
      const ros::TimerEvent&)
  {
    if (!upstream_ok_)
    {
      good_count_ = 0;
      bad_count_++;
      localization_ok_ = false;
      publishState("LOST", false, 0.0f);
      return;
    }

    if (!have_prev_)
    {
      publishState(
          "INIT",
          false,
          0.0f);

      return;
    }

    const double age_sec =
        (ros::WallTime::now() -
         last_pose_wall_time_).toSec();

    if (age_sec > timeout_sec_)
    {
      good_count_ = 0;
      bad_count_++;
      localization_ok_ = false;

      publishState(
          "LOST",
          false,
          0.0f);
    }
  }

  void upstreamHealthCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    upstream_ok_ = msg->data;
    if (!upstream_ok_)
    {
      good_count_ = 0;
      bad_count_ = lost_required_count_;
      localization_ok_ = false;
      publishState("LOST", false, 0.0f);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber upstream_health_sub_;

  ros::Publisher state_pub_;
  ros::Publisher ok_pub_;
  ros::Publisher confidence_pub_;

  ros::Timer timer_;

  std::string global_pose_topic_;
  std::string upstream_health_topic_;
  std::string map_frame_;

  double timeout_sec_;
  double max_jump_m_;
  double max_jump_yaw_deg_;

  int good_required_count_;
  int lost_required_count_;

  bool have_prev_;
  bool upstream_ok_;
  bool localization_ok_;
  int good_count_;
  int bad_count_;

  geometry_msgs::PoseStamped previous_pose_;
  ros::WallTime last_pose_wall_time_;
};

int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_localization_guard");

  LocalizationGuard node;

  ros::spin();

  return 0;
}
