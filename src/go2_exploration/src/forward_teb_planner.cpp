// Exploration-only adapter: align with the path before asking TEB to advance.
// Ordinary navigation continues to load the upstream plugin unchanged.
#include <algorithm>
#include <cmath>
#include <mutex>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <go2_exploration_safety/robot_geometry.h>
#include <tf2/utils.h>
#include "command_envelope.h"
#include "forward_connection.h"

namespace go2_exploration {
class ForwardTebPlanner : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap) override {
    tf_ = tf;
    costmap_ = costmap;
    ros::NodeHandle nh("~/" + name);
    nh.param("max_vel_theta", max_w_, .40);
    nh.param("acc_lim_theta", acc_w_, .30);
    nh.param("xy_goal_tolerance", goal_tolerance_, .15);
    const auto geometry=go2_exploration_safety::RobotGeometry::load();
    front_=geometry.front; rear_=geometry.rear;
    half_width_=geometry.half_width; margin_=geometry.margin;
    ros::param::param("/go2_exploration_safety/min_forward_clearance", forward_clearance_, .18);
    ros::param::param("/go2_exploration_safety/reaction_time", reaction_, .20);
    ros::param::param("/go2_exploration_safety/linear_deceleration", decel_v_, .20);
    ros::param::param("/go2_exploration_safety/angular_deceleration", decel_w_, .40);
    ros::param::param("/go2_exploration_safety/max_prediction_horizon", horizon_, 1.50);
    ros::param::param("/go2_velocity_shaper/max_vx", shaper_.max_v, .30);
    ros::param::param("/go2_velocity_shaper/max_wz", shaper_.max_w, .50);
    ros::param::param("/go2_velocity_shaper/min_sustained_walk_vx", shaper_.walk_floor, .30);
    ros::param::param("/go2_velocity_shaper/min_in_place_wz", shaper_.turn_floor, .50);
    ros::param::param("/go2_velocity_shaper/deadband_vx", shaper_.deadband_v, .015);
    ros::param::param("/go2_velocity_shaper/deadband_wz", shaper_.deadband_w, .01);
    ros::param::param("/go2_sdk_bridge_real/stop_deadband_vx", shaper_.sdk_deadband_v, .025);
    ros::param::param("/go2_sdk_bridge_real/stop_deadband_wz", shaper_.sdk_deadband_w, .01);
    ros::param::param("/go2_sdk_bridge_real/min_turn_wz", shaper_.sdk_turn_floor, .04);
    shaped_sub_=nh.subscribe<geometry_msgs::Twist>("/exploration/cmd_vel_shaped",1,
      [this](const geometry_msgs::Twist::ConstPtr& m) {
        std::lock_guard<std::mutex> lock(shaped_mutex_);
        last_shaped_={m->linear.x,m->angular.z};shaped_stamp_=ros::WallTime::now();
      });
    // Keep the established parameter namespace and local_plan/feedback topics.
    teb_.initialize(name, tf, costmap);
    plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    mode_pub_ = nh.advertise<std_msgs::String>("execution_mode", 1, true);
    last_time_ = ros::WallTime::now();
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    if (plan.empty()) return false;
    if (plan_.empty() || std::hypot(plan.back().pose.position.x-plan_.back().pose.position.x,
                                  plan.back().pose.position.y-plan_.back().pose.position.y) > .10) {
      aligning_ = false;
      last_w_ = 0;
    }
    plan_ = plan;
    return teb_.setPlan(plan);
  }

  bool isGoalReached() override { return !aligning_ && teb_.isGoalReached(); }

  bool computeVelocityCommands(geometry_msgs::Twist& cmd) override {
    cmd = geometry_msgs::Twist();
    geometry_msgs::PoseStamped robot;
    if (plan_.empty() || !costmap_->getRobotPose(robot) || !costmap_->isCurrent())
      return reject("WAITING_FOR_LOCAL_DATA");
    std::vector<geometry_msgs::PoseStamped> local;
    try {
      for (auto p : plan_) {
        p.header.stamp = ros::Time(0);
        local.push_back(tf_->transform(p, robot.header.frame_id, ros::Duration(.02)));
      }
    } catch (const tf2::TransformException&) { return reject("WAITING_FOR_TF"); }
    const auto& r = robot.pose.position;
    size_t nearest = 0;
    double closest = INFINITY;
    for (size_t i=0; i<local.size(); ++i) {
      const double d = std::hypot(local[i].pose.position.x-r.x, local[i].pose.position.y-r.y);
      if (d < closest) { closest=d; nearest=i; }
    }
    size_t lookahead = nearest;
    while (lookahead+1<local.size() &&
           std::hypot(local[lookahead].pose.position.x-r.x,
                      local[lookahead].pose.position.y-r.y) < .45) ++lookahead;
    const double goal_distance = std::hypot(local.back().pose.position.x-r.x,
                                            local.back().pose.position.y-r.y);
    const double heading = goal_distance <= goal_tolerance_
        ? tf2::getYaw(local.back().pose.orientation)
        : std::atan2(local[lookahead].pose.position.y-r.y, local[lookahead].pose.position.x-r.x);
    const double error = wrap(heading-tf2::getYaw(robot.pose.orientation));
    if (std::abs(error) > 1.05 && goal_distance > goal_tolerance_) aligning_ = true;
    if (aligning_ && std::abs(error) < .20) {
      aligning_ = false;
      teb_.setPlan(plan_);
    }
    const auto now = ros::WallTime::now();
    const double dt = std::max(.01, std::min(.20, (now-last_time_).toSec()));
    last_time_ = now;
    if (aligning_) {
      const double wanted = std::copysign(std::min(max_w_, std::sqrt(2*acc_w_*std::abs(error))), error);
      cmd.angular.z = std::max(last_w_-acc_w_*dt, std::min(last_w_+acc_w_*dt, wanted));
    } else if (!teb_.computeVelocityCommands(cmd)) {
      cmd = geometry_msgs::Twist();
      if (connectForward(robot,local,cmd)) return true;
      return reject("TEB_NO_VALID_TRAJECTORY");
    }
    // Reject an unsupported trajectory as a whole. Never turn a reverse arc
    // into a rotation by silently deleting its translational component.
    if (!std::isfinite(cmd.linear.x) || !std::isfinite(cmd.angular.z) || !std::isfinite(cmd.linear.y)) {
      cmd=geometry_msgs::Twist();return reject("INVALID_TEB_COMMAND");
    }
    if (cmd.linear.x < -1e-4 || std::abs(cmd.linear.y) > 1e-4) {
      if (std::abs(error) > .20) aligning_ = true;
      cmd = geometry_msgs::Twist();
      if (connectForward(robot,local,cmd)) return true;
      return reject("REPLAN_UNSUPPORTED_REVERSE");
    }
    if (cmd.linear.x < 0) cmd.linear.x = 0; // numerical noise below 0.1 mm/s
    if (!safeSweep(robot, cmd)) {
      cmd = geometry_msgs::Twist();
      if (connectForward(robot,local,cmd)) return true;
      return reject("LOCAL_STOP_ENVELOPE_BLOCKED");
    }
    last_w_ = cmd.angular.z;
    publishMode(aligning_ ? "ALIGN_TO_PATH" : "TEB_FORWARD");
    if (aligning_) {
      // Publish the actual stationary turn so the existing gate can verify
      // a fresh plan/command handshake and perform its independent veto.
      nav_msgs::Path p;
      p.header = robot.header;
      p.header.stamp = ros::Time::now();
      p.poses.push_back(robot);
      auto end = robot;
      tf2::Quaternion q;
      q.setRPY(0, 0, heading);
      end.pose.orientation = tf2::toMsg(q);
      p.poses.push_back(end);
      plan_pub_.publish(p);
    }
    return true;
  }

 private:
  bool connectForward(const geometry_msgs::PoseStamped& robot,
                      const std::vector<geometry_msgs::PoseStamped>& plan,
                      geometry_msgs::Twist& cmd) {
    const double yaw=tf2::getYaw(robot.pose.orientation),c=std::cos(yaw),s=std::sin(yaw);
    std::vector<RoutePoint> route;
    // Restrict projections to the upcoming portion; later route crossings
    // must not look like immediate progress from the current robot position.
    size_t nearest=0;double distance=INFINITY;
    for(size_t i=0;i<plan.size();++i) {
      const double d=std::hypot(plan[i].pose.position.x-robot.pose.position.x,plan[i].pose.position.y-robot.pose.position.y);
      if(d<distance) {nearest=i;distance=d;}
    }
    double length=0;
    for(size_t i=nearest;i<plan.size();++i) {
      const auto& p=plan[i].pose.position;
      const double dx=p.x-robot.pose.position.x,dy=p.y-robot.pose.position.y;
      route.push_back({c*dx+s*dy,-s*dx+c*dy});
      if(i>nearest)length+=std::hypot(p.x-plan[i-1].pose.position.x,p.y-plan[i-1].pose.position.y);
      if(length>=1.2)break;
    }
    PlanarCommand selected{0,0};
    if(!forwardConnection(route,shaper_,[&](PlanarCommand v) {
          geometry_msgs::Twist trial;trial.linear.x=v.v;trial.angular.z=v.w;
          return safeSweep(robot,trial);
        },selected))return false;
    cmd.linear.x=selected.v;cmd.angular.z=selected.w;last_w_=selected.w;
    // TEB is retried on the next cycle. Publish the actual connection rather
    // than the rejected TEB trajectory for the gate and RViz.
    aligning_=false;
    nav_msgs::Path path;path.header=robot.header;path.header.stamp=ros::Time::now();
    for(int i=0;i<=8;++i) {
      auto pose=robot;const double t=.1*i;const auto p=arcPoint(selected,t);
      pose.header=path.header;pose.pose.position.x+=c*p.x-s*p.y;pose.pose.position.y+=s*p.x+c*p.y;
      tf2::Quaternion q;q.setRPY(0,0,yaw+selected.w*t);pose.pose.orientation=tf2::toMsg(q);path.poses.push_back(pose);
    }
    plan_pub_.publish(path);publishMode("FORWARD_KNOWN_ROUTE_CONNECTION");return true;
  }
  static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
  void publishMode(const std::string& mode) {
    if (mode == mode_) return;
    mode_ = mode;
    std_msgs::String message;
    message.data = mode;
    mode_pub_.publish(message);
  }
  bool reject(const std::string& reason) {
    last_w_ = 0;
    publishMode(reason);
    return false;
  }
  bool safeSweep(const geometry_msgs::PoseStamped& robot, const geometry_msgs::Twist& cmd) {
    auto* map = costmap_->getCostmap();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const double yaw = tf2::getYaw(robot.pose.orientation);
    SweepGeometry geometry;
    geometry.front=front_;geometry.rear=rear_;geometry.half_width=half_width_;geometry.margin=margin_;
    geometry.forward_clearance=forward_clearance_;geometry.reaction=reaction_;
    geometry.decel_v=decel_v_;geometry.decel_w=decel_w_;geometry.horizon=horizon_;
    geometry.origin_x=map->getOriginX();geometry.origin_y=map->getOriginY();
    geometry.width=map->getSizeInCellsX()*map->getResolution();geometry.height=map->getSizeInCellsY()*map->getResolution();
    const auto free_cell=[map](double x,double y) {
      unsigned mx,my;
      return map->worldToMap(x,y,mx,my) && map->getCost(mx,my)<costmap_2d::LETHAL_OBSTACLE;
    };
    const auto clear=[&](PlanarCommand v) {
      return sweepClear(robot.pose.position.x,robot.pose.position.y,yaw,v,geometry,map->getResolution(),free_cell);
    };
    // Include intermediate shaped speeds, whose turn radius may be tighter
    // than both the raw and steady command. The gate checks actual output too.
    const PlanarCommand raw{cmd.linear.x,cmd.angular.z};
    const auto target=shapedTarget(raw,shaper_);
    if(!clear(raw) || !clear(target))return false;
    std::lock_guard<std::mutex> shaped_lock(shaped_mutex_);
    const auto current=shaped_stamp_.isZero() || (ros::WallTime::now()-shaped_stamp_).toSec()>.25 ? PlanarCommand{0,0} : last_shaped_;
    return transitionClear(current,target,shaper_,clear);
  }
  teb_local_planner::TebLocalPlannerROS teb_;
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ = nullptr;
  std::vector<geometry_msgs::PoseStamped> plan_;
  ros::Publisher plan_pub_, mode_pub_;
  ros::WallTime last_time_;
  std::string mode_;
  bool aligning_ = false;
  double last_w_ = 0, max_w_, acc_w_, goal_tolerance_;
  double front_, rear_, half_width_, margin_, forward_clearance_;
  double reaction_, decel_v_, decel_w_, horizon_;
  ShaperLimits shaper_;
  ros::Subscriber shaped_sub_;
  std::mutex shaped_mutex_;
  PlanarCommand last_shaped_{0,0};
  ros::WallTime shaped_stamp_;
};
} // namespace go2_exploration
PLUGINLIB_EXPORT_CLASS(go2_exploration::ForwardTebPlanner, nav_core::BaseLocalPlanner)
