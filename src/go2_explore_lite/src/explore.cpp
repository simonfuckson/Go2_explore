/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>
#include <go2_exploration_safety/robot_geometry.h>
#include <explore/boundary_goal.h>
#include <explore/known_space.h>
#include <explore/anchored_path.h>
#include <explore/view_gain.h>
#include <explore/arrival_view.h>
#include <explore/candidate_search.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/String.h>
#include <sstream>
#include <limits>
#include <set>
#include <iomanip>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , geometry_nh_("~geometry")
  , local_nh_("~local_geometry")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , geometry_client_(geometry_nh_, relative_nh_, &tf_listener_)
  , local_client_(local_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("preview_only", preview_only_, false);
  double xy_tolerance = .15;
  relative_nh_.param("/move_base/TebLocalPlannerROS/xy_goal_tolerance", xy_tolerance, .15);
  private_nh_.param("minimum_goal_distance", minimum_goal_distance_, .40);
  minimum_goal_distance_ = std::max(minimum_goal_distance_,
      xy_tolerance + costmap_client_.getCostmap()->getResolution());
  selected_goal_publisher_ = private_nh_.advertise<geometry_msgs::PoseStamped>("selected_goal", 1, true);
  selection_status_publisher_=private_nh_.advertise<std_msgs::String>("selection_status",1,true);
  view_publisher_=private_nh_.advertise<visualization_msgs::MarkerArray>("view_candidates",1,true);
  private_nh_.param("view_range",view_range_,3.0);
  if (!relative_nh_.getParam("/go2_terrain_guard/sensor_height",view_height_))
    throw std::runtime_error("GO2 terrain sensor height is missing");
  double pitch_deg;
  if (!relative_nh_.getParam("/mid360_mount/base_link_to_lidar_link/pitch_deg",pitch_deg))
    throw std::runtime_error("GO2 MID360 extrinsics are missing");
  view_pitch_=pitch_deg*M_PI/180.;
  if (!relative_nh_.getParam("/mid360_mount/base_link_to_lidar_link/x",sensor_x_) ||
      !relative_nh_.getParam("/mid360_mount/base_link_to_lidar_link/y",sensor_y_) ||
      !relative_nh_.getParam("/mid360_mount/base_link_to_lidar_link/yaw_deg",sensor_yaw_))
    throw std::runtime_error("GO2 MID360 sensor origin/orientation is missing");
  sensor_yaw_*=M_PI/180.;
  if (!relative_nh_.getParam("/mid360_mount/base_link_to_lidar_link/roll_deg",sensor_roll_))
    throw std::runtime_error("GO2 lidar roll is missing");
  sensor_roll_*=M_PI/180.;
  const auto geometry=go2_exploration_safety::RobotGeometry::load();
  front_=geometry.front+geometry.margin;
  rear_=geometry.rear+geometry.margin;
  half_width_=geometry.half_width+geometry.margin;
  stop_front_=front_+geometry.clearance;
  private_nh_.param("candidate_retry_seconds",retry_seconds_,15.0);
  private_nh_.param("failed_goal_retry_seconds",blacklist_seconds_,30.0);
  // Supervisor preserves interrupted targets across a stopped/restarted producer.
  XmlRpc::XmlRpcValue recovery_blacklist;
  if (private_nh_.getParam("recovery_blacklist", recovery_blacklist) &&
      recovery_blacklist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i=0; i<recovery_blacklist.size(); ++i) {
      auto& entry=recovery_blacklist[i];
      if (entry.getType()!=XmlRpc::XmlRpcValue::TypeStruct ||
          !entry.hasMember("x") || !entry.hasMember("y") ||
          !entry.hasMember("until") || !entry.hasMember("frame") ||
          entry["x"].getType()!=XmlRpc::XmlRpcValue::TypeDouble ||
          entry["y"].getType()!=XmlRpc::XmlRpcValue::TypeDouble ||
          entry["until"].getType()!=XmlRpc::XmlRpcValue::TypeDouble ||
          entry["frame"].getType()!=XmlRpc::XmlRpcValue::TypeString) continue;
      const double until=static_cast<double>(entry["until"]);
      if (!std::isfinite(until) || until<=ros::Time::now().toSec() ||
          static_cast<std::string>(entry["frame"])!=costmap_client_.getGlobalFrameID()) continue;
      geometry_msgs::Point point;
      point.x=static_cast<double>(entry["x"]); point.y=static_cast<double>(entry["y"]);
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
      frontier_blacklist_.push_back(point);
      blacklist_until_.push_back(ros::Time(until));
    }
  }
  private_nh_.param("plan_checks_per_cycle",plan_budget_,12);
  plan_budget_=std::max(1,std::min(24,plan_budget_));
  private_nh_.param("selection_budget_ms",cycle_budget_ms_,75.);
  private_nh_.param("goal_overhead_seconds",goal_overhead_seconds_,2.);
  relative_nh_.param("/go2_exploration_safety/max_linear_speed",scoring_speed_,.30);
  relative_nh_.param("/go2_exploration_safety/max_angular_speed",scoring_yaw_speed_,.50);
  if(!std::isfinite(cycle_budget_ms_)||cycle_budget_ms_<5.||cycle_budget_ms_>250.||
     !std::isfinite(goal_overhead_seconds_)||goal_overhead_seconds_<=0.||
     !std::isfinite(scoring_speed_)||scoring_speed_<=0.||
     !std::isfinite(scoring_yaw_speed_)||scoring_yaw_speed_<=0.)
    throw std::runtime_error("Invalid exploration selection budget/scoring parameters");
  make_plan_client_ = relative_nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, geometry_client_.getCostmap());

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  if (!costmap_client_.isFresh(2.5) || !geometry_client_.isFresh(2.5)) return;
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  const auto& q = pose.orientation;
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < .5) {
    ROS_WARN_THROTTLE(2.0, "No valid robot pose; not selecting an exploration goal");
    return;
  }
  if (goal_active_) {
    const double distance = std::hypot(prev_goal_.x-pose.position.x, prev_goal_.y-pose.position.y);
    if (distance < prev_distance_ - .02) {
      last_progress_ = ros::Time::now();
      prev_distance_ = distance;
    }
    if (ros::Time::now()-last_progress_ <= progress_timeout_) return;
    blacklistGoal(prev_goal_);
    move_base_client_.cancelGoal();
    goal_active_ = false;
    ROS_WARN("Exploration goal made no progress; waiting for cancellation before selecting another");
    return;
  }
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    // Map refresh can temporarily remove frontiers. Keep the timer alive;
    // the session supervisor owns the multi-map completion decision.
    waiting_since_=ros::WallTime();
    std_msgs::String status;status.data="WAITING reason=no_frontier_tasks frontiers=0 checked=0";
    selection_status_publisher_.publish(status);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  geometry_msgs::Point target_position;
  if (!selectBoundaryGoal(frontiers, pose, target_position)) {
    ROS_WARN_THROTTLE(5.0, "Frontiers exist but no boundary goal outside arrival tolerance has a valid plan; waiting for map update");
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw_);
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  selected_goal_publisher_.publish(goal.target_pose);
  ROS_INFO_THROTTLE(2.0, "Boundary goal (%.3f, %.3f), robot distance %.3f m, preview=%s",
      target_position.x, target_position.y,
      std::hypot(target_position.x-pose.position.x, target_position.y-pose.position.y),
      preview_only_ ? "true" : "false");
  if (preview_only_) return;
  prev_goal_ = target_position;
  prev_distance_ = std::hypot(target_position.x-pose.position.x, target_position.y-pose.position.y);
  last_progress_ = ros::Time::now();
  goal_active_ = true;
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::selectBoundaryGoal(
    const std::vector<frontier_exploration::Frontier>& frontiers,
    const geometry_msgs::Pose& pose, geometry_msgs::Point& target)
{
  const auto cycle_start=ros::WallTime::now();
  const double resolution = costmap_client_.getCostmap()->getResolution();
  auto* geometry = geometry_client_.getCostmap();
  auto* local_map = local_client_.getCostmap();
  if (geometry_client_.getGlobalFrameID() != costmap_client_.getGlobalFrameID()) return false;
  if (!geometry_client_.isFresh(2.5) || !local_client_.isFresh(.75)) return false;
  tf::StampedTransform map_to_local;
  try {
    tf_listener_.lookupTransform(local_client_.getGlobalFrameID(),
        costmap_client_.getGlobalFrameID(), ros::Time(0), map_to_local);
  } catch (const tf::TransformException&) { return false; }
  if (frontiers.empty()) return false;
  const auto now=ros::Time::now();
  const double robot_yaw=tf::getYaw(pose.orientation);
  for (auto it=retries_.begin();it!=retries_.end();) {
    unsigned mx,my;
    const auto& entry=it->second;
    auto* map=entry.local?local_map:geometry;
    const bool changed=map->worldToMap(entry.blocked_x,entry.blocked_y,mx,my)
        && map->getCost(mx,my)!=entry.blocked_cost;
    if (entry.until<=now || changed || retryPoseChanged(pose.position.x,pose.position.y,robot_yaw,
          entry.robot_x,entry.robot_y,entry.robot_yaw)) it=retries_.erase(it); else ++it;
  }
  std::vector<std::vector<geometry_msgs::Point>> queues;
  std::vector<CandidateKey> queue_keys;
  std::set<CandidateKey> active_keys;
  for (const auto& frontier:frontiers) {
    auto candidates=boundaryCandidates(knownApproaches(*geometry,frontier.points),
        frontier.centroid,pose.position,robot_yaw,minimum_goal_distance_);
    if(queues.empty()) {
      // With a long body and a forward-looking ground sensor, reaching a
      // frontier may require first advancing inside the observed corridor.
      // Boundary-only candidates can all demand an impossible starting turn.
      // These are goals only: they pass the same gain, complete footprint,
      // global-plan, local-stop and arrival-rotation checks below.
      std::vector<geometry_msgs::Point> approaches;
      for(const double distance:{.60,.80,1.00})for(const double angle:{0.,-.15,.15}) {
        if(distance<minimum_goal_distance_)continue;
        geometry_msgs::Point p;
        p.x=pose.position.x+distance*std::cos(robot_yaw+angle);
        p.y=pose.position.y+distance*std::sin(robot_yaw+angle);
        approaches.push_back(p);
      }
      candidates.insert(candidates.begin(),approaches.begin(),approaches.end());
    }
    queues.push_back(diverseCandidates(candidates,resolution));
    const auto key=candidateKey(frontier.initial.x,frontier.initial.y,resolution);
    queue_keys.push_back(key);active_keys.insert(key);
  }
  for(auto it=candidate_cursors_.begin();it!=candidate_cursors_.end();)
    if(!active_keys.count(it->first))it=candidate_cursors_.erase(it);else ++it;
  std::vector<CandidateCursor> positions;
  size_t candidate_count=0;
  for(size_t i=0;i<queues.size();++i) {
    positions.emplace_back(candidate_cursors_[queue_keys[i]],queues[i].size());
    candidate_count+=queues[i].size();
  }
  std::map<std::string,int> rejected;
  int calls=0, examined=0;
  bool found=false;
  double best_score=std::numeric_limits<double>::infinity();
  double selected_gain=0,selected_length=0,selected_turn=0;
  geometry_msgs::Point last_blocked;
  std::string blocked_frame,last_failure;
  unsigned char last_blocked_cost=0;
  double blocked_distance=0.;
  visualization_msgs::MarkerArray views;
  visualization_msgs::Marker clear;
  clear.action=visualization_msgs::Marker::DELETEALL; views.markers.push_back(clear);
  size_t cursor=frontier_cursor_%queues.size();
  int exhausted=0;
  while (calls<plan_budget_ && examined<240 && exhausted<static_cast<int>(queues.size())) {
      if(examined>0 && (ros::WallTime::now()-cycle_start).toSec()*1000.>=cycle_budget_ms_)break;
      const size_t index=cursor;
      cursor=(cursor+1)%queues.size();
      if (positions[index].empty()) {++exhausted;continue;}
      exhausted=0; ++examined;
      const auto candidate=queues[index][positions[index].pop()];
      const auto key=candidateKey(candidate.x,candidate.y,resolution);
      if (goalOnBlacklist(candidate)) {++rejected["failed_goal_cooldown"];continue;}
      const auto cached=retries_.find(key);
      if(cached!=retries_.end()) {++rejected["cached_"+cached->second.reason];continue;}
      const double toward=std::atan2(frontiers[index].centroid.y-candidate.y,
                                     frontiers[index].centroid.x-candidate.x);
      const double approach=std::atan2(candidate.y-pose.position.y,candidate.x-pose.position.x);
      std::vector<double> headings{toward,toward-.785398163,toward+.785398163,robot_yaw,approach};
      double view_yaw=toward, gain=-1;
      geometry_msgs::Point preblocked=candidate;
      unsigned char preblocked_cost=costmap_2d::FREE_SPACE;
      for (const double heading:headings) {
        if (!knownFootprint(*geometry,candidate.x,candidate.y,heading,stop_front_,rear_,half_width_,&preblocked,&preblocked_cost)) continue;
        const double value=expectedGroundGain(*geometry,candidate.x,candidate.y,heading,
                                             view_range_,view_height_,view_pitch_,sensor_x_,sensor_y_,sensor_yaw_,sensor_roll_,costmap_client_.getCostmap());
        if (value>gain) {gain=value;view_yaw=heading;}
      }
      if (gain<.01) {
        const std::string reason=gain<0?"goal_footprint":"no_view_gain";
        ++rejected[reason];
        // Visibility changes as the coverage map updates, without a change
        // at the candidate's collision cell. Revisit these views promptly.
        const double delay=gain<0?retry_seconds_:std::min(2.,retry_seconds_);
        retries_[key]={now+ros::Duration(delay),preblocked.x,preblocked.y,preblocked_cost,false,
                      pose.position.x,pose.position.y,robot_yaw,reason};
        continue;
      }
      ++calls;
      nav_msgs::GetPlan request;
      request.request.start.header.frame_id = costmap_client_.getGlobalFrameID();
      request.request.start.header.stamp = ros::Time::now();
      request.request.start.pose = pose;
      request.request.goal = request.request.start;
      request.request.goal.pose.position = candidate;
      request.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(view_yaw);
      request.request.tolerance = 0.0;
      geometry_msgs::Point blocked=candidate;
      unsigned char blocked_cost=costmap_2d::FREE_SPACE;
      bool blocked_local=false;
      double path_blocked_distance=0.;
      unsigned mx,my;
      if (geometry->worldToMap(candidate.x,candidate.y,mx,my)) blocked_cost=geometry->getCost(mx,my);
      auto defer=[&](const std::string& reason) {
        ++rejected[reason];
        retries_[key]={now+ros::Duration(retry_seconds_),blocked.x,blocked.y,blocked_cost,blocked_local,
                      pose.position.x,pose.position.y,robot_yaw,reason};
        last_failure=reason;last_blocked=blocked;last_blocked_cost=blocked_cost;
        blocked_frame=blocked_local?local_client_.getGlobalFrameID():geometry_client_.getGlobalFrameID();
        blocked_distance=path_blocked_distance;
      };
      if (!make_plan_client_.call(request) || request.response.plan.poses.empty()) {defer("no_global_plan");continue;}
      const auto& end = request.response.plan.poses.back().pose.position;
      if (std::hypot(end.x-candidate.x,end.y-candidate.y) > resolution ||
          std::hypot(end.x-pose.position.x,end.y-pose.position.y) < minimum_goal_distance_) {defer("plan_endpoint");continue;}
      bool executable = true;
      const auto path = anchoredPath(request.response.plan.poses,pose,resolution);
      if (path.empty()) {defer("invalid_global_plan");continue;}
      std::string failure="global_footprint";
      double final_heading=robot_yaw, length=0,turn=0;
      for (size_t i=0; i<path.size(); ++i) {
        const auto& p=path[i];
        const auto& next=path[std::min(i+1,path.size()-1)];
        final_heading=p.yaw;
        length+=std::hypot(next.x-p.x,next.y-p.y);
        turn+=std::abs(angleDifference(next.yaw,p.yaw));
        path_blocked_distance=std::hypot(p.x-pose.position.x,p.y-pose.position.y);
        if (!knownFootprint(*geometry,p.x,p.y,final_heading,front_,rear_,half_width_,&blocked,&blocked_cost)) { executable=false; break; }
        if (std::hypot(p.x-pose.position.x,p.y-pose.position.y) <= .75) {
          const auto lp=map_to_local*tf::Vector3(p.x,p.y,0);
          if (!knownFootprint(*local_map,lp.x(),lp.y(),
                             final_heading+tf::getYaw(map_to_local.getRotation()),stop_front_,rear_,half_width_,&blocked,&blocked_cost)) {
            blocked_local=true;failure="local_stop_envelope"; executable=false; break;
          }
        }
      }
      // A view includes its arrival rotation; do not ask TEB to turn into a
      // blind/occupied swept region after arriving at an otherwise valid point.
      headings.push_back(final_heading);
      if(executable && !feasibleArrivalView(headings,final_heading,[&](double heading) {
          return knownFootprint(*geometry,end.x,end.y,heading,stop_front_,rear_,half_width_,&blocked,&blocked_cost);
        },[&](double heading) {
          return expectedGroundGain(*geometry,candidate.x,candidate.y,heading,view_range_,view_height_,view_pitch_,sensor_x_,sensor_y_,sensor_yaw_,sensor_roll_,costmap_client_.getCostmap());
        },view_yaw,gain)) {failure="arrival_rotation";executable=false;}
      visualization_msgs::Marker arrow;
      arrow.header.frame_id=geometry_client_.getGlobalFrameID(); arrow.header.stamp=now;
      arrow.ns="candidate_views";arrow.id=calls;arrow.type=visualization_msgs::Marker::ARROW;
      arrow.action=visualization_msgs::Marker::ADD;
      arrow.pose.position=candidate;arrow.pose.position.z=.1;
      arrow.pose.orientation=tf::createQuaternionMsgFromYaw(view_yaw);
      arrow.scale.x=.45;arrow.scale.y=.05;arrow.scale.z=.10;
      arrow.color.a=.9;arrow.color.r=executable?.2:1.;arrow.color.g=executable?1.:.15;
      arrow.lifetime=ros::Duration(3.);views.markers.push_back(arrow);
      if (!executable) {defer(failure);continue;}
      turn+=std::abs(angleDifference(view_yaw,final_heading));
      const double score=-informationRate(gain,length,turn,scoring_speed_,scoring_yaw_speed_,goal_overhead_seconds_);
      if (score<best_score) {
        best_score=score;target_yaw_=view_yaw;target=candidate;found=true;
        selected_gain=gain;selected_length=length;selected_turn=turn;
      }
  }
  frontier_cursor_=(cursor+1)%queues.size();
  for(size_t i=0;i<queues.size();++i)candidate_cursors_[queue_keys[i]]=positions[i].next;
  if(found)waiting_since_=ros::WallTime();
  else if(waiting_since_.isZero())waiting_since_=cycle_start;
  std::ostringstream status;
  status<<std::fixed<<std::setprecision(3);
  status<<(found?"SELECTED":"WAITING")<<" frontiers="<<frontiers.size()<<" checked="<<calls<<" examined="<<examined
        <<" candidates="<<candidate_count<<" selection_ms="<<(ros::WallTime::now()-cycle_start).toSec()*1000.;
  if(found)status<<" gain_m2="<<selected_gain<<" route_m="<<selected_length
                 <<" turn_rad="<<selected_turn<<" gain_per_second="<<-best_score;
  else status<<" waiting_s="<<(cycle_start-waiting_since_).toSec()
             <<" reason="<<(last_failure.empty()?"candidate_search_pending":last_failure);
  if(!last_failure.empty())status<<" blocked_frame="<<blocked_frame<<" blocked_x="<<last_blocked.x
      <<" blocked_y="<<last_blocked.y<<" blocked_cost="<<static_cast<int>(last_blocked_cost)
      <<" blocked_path_distance_m="<<blocked_distance;
  for (const auto& entry:rejected) status<<" "<<entry.first<<"="<<entry.second;
  std_msgs::String message;message.data=status.str();selection_status_publisher_.publish(message);
  view_publisher_.publish(views);
  return found;
}

void Explore::blacklistGoal(const geometry_msgs::Point& goal) {
  frontier_blacklist_.push_back(goal);
  blacklist_until_.push_back(ros::Time::now()+ros::Duration(blacklist_seconds_));
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (size_t i=0;i<frontier_blacklist_.size();++i) {
    if (blacklist_until_[i]<=ros::Time::now()) continue;
    const auto& frontier_goal=frontier_blacklist_[i];
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (!(frontier_goal == prev_goal_)) return;
  goal_active_ = false;
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    blacklistGoal(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  if (!preview_only_) move_base_client_.cancelGoal();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  explore::Explore explore;
  ros::spin();

  return 0;
}
