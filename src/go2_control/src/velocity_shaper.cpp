#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

class VelocityShaper {
 public:
  VelocityShaper() : nh_(), pnh_("~") {
    pnh_.param("max_vx", max_vx_, 0.60);
    pnh_.param("min_sustained_walk_vx", min_sustained_walk_vx_, 0.30);
    pnh_.param("max_reverse_vx", max_reverse_vx_, 0.18);
    pnh_.param("min_sustained_reverse_vx",
               min_sustained_reverse_vx_, 0.12);
    pnh_.param("max_wz", max_wz_, 0.80);
    pnh_.param("min_in_place_wz", min_in_place_wz_, 0.50);
    pnh_.param("max_ax", max_ax_, 0.60);
    pnh_.param("max_awz", max_awz_, 0.80);
    pnh_.param("timeout_sec", timeout_sec_, 0.50);
    pnh_.param("deadband_vx", deadband_vx_, 0.015);
    pnh_.param("deadband_wz", deadband_wz_, 0.01);
    pnh_.param("yaw_reverse_hold_sec", yaw_reverse_hold_sec_, 0.20);
    pnh_.param("pure_turn_direction_lock_sec",
               pure_turn_direction_lock_sec_, 3.0);
    pnh_.param("pure_turn_zero_release_sec",
               pure_turn_zero_release_sec_, 0.35);
    pnh_.param("enforce_pure_turn_output_floor",
               enforce_pure_turn_output_floor_, true);
    pnh_.param("allow_reverse", allow_reverse_, true);
    pnh_.param("rate_hz", rate_hz_, 50.0);
    pnh_.param("require_terrain_health", require_terrain_health_, false);
    pnh_.param("terrain_health_timeout_sec", terrain_health_timeout_sec_,
               0.75);
    pnh_.param<std::string>("terrain_health_topic", terrain_health_topic_,
                            "/terrain/healthy");
    if (terrain_health_timeout_sec_ <= 0.0) {
      throw std::invalid_argument(
          "terrain_health_timeout_sec must be positive");
    }
    min_sustained_walk_vx_ = clamp(min_sustained_walk_vx_, 0.0, max_vx_);
    max_reverse_vx_ = clamp(max_reverse_vx_, 0.0, max_vx_);
    min_sustained_reverse_vx_ = clamp(
        min_sustained_reverse_vx_, 0.0, max_reverse_vx_);
    min_in_place_wz_ = clamp(min_in_place_wz_, 0.0, max_wz_);

    command_sub_ = nh_.subscribe("/cmd_vel_nav", 10,
        &VelocityShaper::commandCallback, this);
    localization_sub_ = nh_.subscribe("/localization/ok", 10,
        &VelocityShaper::localizationCallback, this);
    if (require_terrain_health_) {
      terrain_health_sub_ = nh_.subscribe(
          terrain_health_topic_, 1,
          &VelocityShaper::terrainHealthCallback, this);
    }
    command_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_safe", 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz_),
        &VelocityShaper::timerCallback, this);
    last_tick_ = ros::WallTime::now();
  }

 private:
  static double clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
  }

  static bool finite(const geometry_msgs::Twist& msg) {
    return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
           std::isfinite(msg.angular.z);
  }

  bool terrainPermitted(const ros::WallTime& now) const {
    return !require_terrain_health_ ||
        (have_terrain_health_ && terrain_healthy_ &&
         (now - last_terrain_health_).toSec() <=
             terrain_health_timeout_sec_);
  }

  void stopImmediately(const char* reason) {
    target_ = geometry_msgs::Twist();
    current_ = geometry_msgs::Twist();
    have_command_ = false;
    clearPureTurnDirectionLock(reason);
    pure_turn_zero_since_ = ros::WallTime();
    yaw_zero_since_ = ros::WallTime::now();
    if (command_pub_) {
      command_pub_.publish(current_);
    }
  }

  void clearPureTurnDirectionLock(const char* reason) {
    if (pure_turn_direction_ == 0) return;
    ROS_INFO("Pure-turn direction lock cleared: %s", reason);
    pure_turn_direction_ = 0;
    pure_turn_lock_started_ = ros::WallTime();
  }

  void commandCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!finite(*msg)) {
      ROS_ERROR_THROTTLE(1.0, "Rejected non-finite /cmd_vel_nav");
      return;
    }
    if (!terrainPermitted(ros::WallTime::now())) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected /cmd_vel_nav while terrain health gate is closed");
      stopImmediately("terrain health gate closed");
      return;
    }
    target_ = *msg;
    if (target_.linear.x >= 0.0) {
      target_.linear.x = clamp(target_.linear.x, 0.0, max_vx_);
    } else if (allow_reverse_) {
      target_.linear.x = clamp(target_.linear.x, -max_reverse_vx_, 0.0);
    } else {
      ROS_WARN_THROTTLE(1.0,
          "Rejected reverse command vx=%.3f because allow_reverse=false",
          target_.linear.x);
      target_.linear.x = 0.0;
    }
    if (target_.linear.x >= deadband_vx_ &&
        target_.linear.x < min_sustained_walk_vx_) {
      target_.linear.x = std::copysign(min_sustained_walk_vx_,
                                       target_.linear.x);
    } else if (target_.linear.x <= -deadband_vx_ &&
               std::fabs(target_.linear.x) < min_sustained_reverse_vx_) {
      target_.linear.x = -min_sustained_reverse_vx_;
    }
    target_.linear.y = 0.0;
    target_.linear.z = 0.0;
    target_.angular.x = 0.0;
    target_.angular.y = 0.0;
    target_.angular.z = clamp(target_.angular.z, -max_wz_, max_wz_);
    if (std::fabs(target_.angular.z) < deadband_wz_) {
      target_.angular.z = 0.0;
    } else if (std::fabs(target_.linear.x) < deadband_vx_ &&
               std::fabs(target_.angular.z) < min_in_place_wz_) {
      target_.angular.z = std::copysign(min_in_place_wz_,
                                        target_.angular.z);
    }
    const ros::WallTime now = ros::WallTime::now();
    const bool has_translation =
        std::fabs(target_.linear.x) >= deadband_vx_;
    const bool has_turn =
        std::fabs(target_.angular.z) >= deadband_wz_;
    if (has_translation) {
      pure_turn_zero_since_ = ros::WallTime();
      clearPureTurnDirectionLock("translation resumed");
    } else if (!has_turn) {
      if (pure_turn_zero_since_.isZero()) pure_turn_zero_since_ = now;
      if ((now - pure_turn_zero_since_).toSec() >=
          pure_turn_zero_release_sec_) {
        clearPureTurnDirectionLock("sustained zero turn command");
      }
    } else if (pure_turn_direction_lock_sec_ > 0.0) {
      pure_turn_zero_since_ = ros::WallTime();
      const int requested_direction = target_.angular.z > 0.0 ? 1 : -1;
      const bool lock_expired = pure_turn_direction_ != 0 &&
          (now - pure_turn_lock_started_).toSec() >=
              pure_turn_direction_lock_sec_;
      if (pure_turn_direction_ == 0 || lock_expired) {
        pure_turn_direction_ = requested_direction;
        pure_turn_lock_started_ = now;
        ROS_INFO("Pure-turn direction locked %s for %.2f sec",
                 pure_turn_direction_ > 0 ? "CCW" : "CW",
                 pure_turn_direction_lock_sec_);
      } else if (requested_direction != pure_turn_direction_) {
        target_.angular.z = std::copysign(
            std::fabs(target_.angular.z),
            static_cast<double>(pure_turn_direction_));
        ROS_WARN_THROTTLE(
            1.0,
            "Rejected rapid pure-turn reversal; holding %s for %.2f sec more",
            pure_turn_direction_ > 0 ? "CCW" : "CW",
            std::max(0.0, pure_turn_direction_lock_sec_ -
                (now - pure_turn_lock_started_).toSec()));
      }
    }
    last_command_ = now;
    have_command_ = true;
  }

  void localizationCallback(const std_msgs::Bool::ConstPtr& msg) {
    localization_ok_ = msg->data;
    if (!localization_ok_) {
      target_ = geometry_msgs::Twist();
      current_ = geometry_msgs::Twist();
      clearPureTurnDirectionLock("localization not OK");
      pure_turn_zero_since_ = ros::WallTime();
    }
  }

  void terrainHealthCallback(const std_msgs::Bool::ConstPtr& msg) {
    last_terrain_health_ = ros::WallTime::now();
    have_terrain_health_ = true;
    terrain_healthy_ = msg->data;
    if (!terrain_healthy_) {
      ROS_ERROR_THROTTLE(
          1.0, "Terrain health false: velocity output stopped immediately");
      stopImmediately("terrain health false");
      terrain_gate_was_open_ = false;
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    const double dt = std::max(0.001, (now - last_tick_).toSec());
    last_tick_ = now;

    const bool terrain_permitted = terrainPermitted(now);
    if (!terrain_permitted) {
      if (terrain_gate_was_open_) {
        ROS_ERROR(
            "Terrain health heartbeat unavailable or stale: velocity output stopped");
      }
      terrain_gate_was_open_ = false;
      stopImmediately("terrain health unavailable or stale");
      return;
    }
    if (require_terrain_health_ && !terrain_gate_was_open_) {
      ROS_INFO("Terrain health gate opened; waiting for a fresh velocity command");
    }
    terrain_gate_was_open_ = true;

    const bool fresh = have_command_ &&
        (now - last_command_).toSec() <= timeout_sec_;
    if (!localization_ok_ || !fresh) {
      target_ = geometry_msgs::Twist();
      if (!fresh) {
        clearPureTurnDirectionLock("command timeout");
        pure_turn_zero_since_ = ros::WallTime();
      }
      if (!localization_ok_) {
        current_ = geometry_msgs::Twist();
      }
    }

    current_.linear.x += clamp(target_.linear.x - current_.linear.x,
                               -max_ax_ * dt, max_ax_ * dt);
    double yaw_target = target_.angular.z;
    const bool reversing = current_.angular.z * yaw_target < 0.0;
    if (reversing) {
      yaw_target = 0.0;
      yaw_zero_since_ = ros::WallTime();
    } else if (std::fabs(current_.angular.z) < deadband_wz_) {
      current_.angular.z = 0.0;
      if (yaw_zero_since_.isZero()) yaw_zero_since_ = now;
      if (!yaw_zero_since_.isZero() &&
          (now - yaw_zero_since_).toSec() < yaw_reverse_hold_sec_) {
        yaw_target = 0.0;
      }
    } else {
      yaw_zero_since_ = ros::WallTime();
    }
    current_.angular.z += clamp(yaw_target - current_.angular.z,
                                -max_awz_ * dt, max_awz_ * dt);

    if (enforce_pure_turn_output_floor_) {
      const bool pure_turn_target =
          std::fabs(target_.linear.x) < deadband_vx_ &&
          std::fabs(target_.angular.z) >= min_in_place_wz_;
      const bool below_stepping_rate =
          std::fabs(current_.angular.z) >= deadband_wz_ &&
          std::fabs(current_.angular.z) < min_in_place_wz_;

      // Do not dwell in the low-yaw body-twist range.  Once the reversal hold
      // has elapsed, enter the controller's stepping range immediately.
      if (!reversing && pure_turn_target &&
          std::fabs(yaw_target) >= min_in_place_wz_ &&
          below_stepping_rate) {
        current_.angular.z = std::copysign(min_in_place_wz_, yaw_target);
      }

      // On stop or reversal, leave the stepping range directly instead of
      // ramping through low yaw rates that can drag loaded feet.
      const bool stopping_turn =
          std::fabs(target_.angular.z) < deadband_wz_ || reversing;
      if (stopping_turn && below_stepping_rate) {
        current_.angular.z = 0.0;
        yaw_zero_since_ = now;
      }
    }
    current_.linear.y = 0.0;

    if (std::fabs(current_.linear.x) < deadband_vx_ &&
        std::fabs(target_.linear.x) < deadband_vx_) {
      current_.linear.x = 0.0;
    }
    if (std::fabs(current_.angular.z) < deadband_wz_ &&
        std::fabs(target_.angular.z) < deadband_wz_) {
      current_.angular.z = 0.0;
    }
    command_pub_.publish(current_);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber command_sub_, localization_sub_, terrain_health_sub_;
  ros::Publisher command_pub_;
  ros::Timer timer_;
  geometry_msgs::Twist target_, current_;
  ros::WallTime last_command_, last_tick_, yaw_zero_since_;
  ros::WallTime pure_turn_lock_started_;
  ros::WallTime pure_turn_zero_since_;
  ros::WallTime last_terrain_health_;
  bool localization_ok_ = false;
  bool have_command_ = false;
  bool allow_reverse_ = true;
  bool enforce_pure_turn_output_floor_ = true;
  bool require_terrain_health_ = false;
  bool have_terrain_health_ = false;
  bool terrain_healthy_ = false;
  bool terrain_gate_was_open_ = false;
  double max_vx_, min_sustained_walk_vx_;
  double max_reverse_vx_, min_sustained_reverse_vx_;
  double max_wz_, min_in_place_wz_;
  double max_ax_, max_awz_;
  double timeout_sec_, deadband_vx_, deadband_wz_, yaw_reverse_hold_sec_;
  double pure_turn_direction_lock_sec_;
  double pure_turn_zero_release_sec_;
  double rate_hz_;
  double terrain_health_timeout_sec_ = 0.75;
  std::string terrain_health_topic_ = "/terrain/healthy";
  int pure_turn_direction_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_velocity_shaper");
  VelocityShaper node;
  ros::spin();
  return 0;
}
