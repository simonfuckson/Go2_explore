#include <cstdint>
#include <string>
#include <future>
#include <chrono>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <boost/bind/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

class NavigationSupervisor {
 public:
  using MoveBaseActionServer =
      actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>;
  using MoveBaseActionClient =
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  NavigationSupervisor()
      : nh_(),
        pnh_("~"),
        external_action_server_(nh_, "/move_base", false),
        internal_action_client_("/move_base_internal", false) {
    pnh_.param("require_terrain_health", require_terrain_health_, false);
    pnh_.param("terrain_health_timeout_sec", terrain_health_timeout_sec_, 0.75);
    if (terrain_health_timeout_sec_ <= 0.0) {
      ROS_WARN("Invalid terrain_health_timeout_sec; using 0.75 seconds");
      terrain_health_timeout_sec_ = 0.75;
    }

    localization_sub_ = nh_.subscribe(
        "/localization/ok", 10,
        &NavigationSupervisor::localizationCallback, this);
    control_enabled_sub_ = nh_.subscribe(
        "/go2/control/enabled", 10,
        &NavigationSupervisor::controlEnabledCallback, this);
    control_state_sub_ = nh_.subscribe(
        "/go2/control/state", 10,
        &NavigationSupervisor::controlStateCallback, this);
    simple_goal_sub_ = nh_.subscribe(
        "/move_base_simple/goal", 1,
        &NavigationSupervisor::simpleGoalCallback, this);
    public_cancel_sub_ = nh_.subscribe(
        "/move_base/cancel", 10,
        &NavigationSupervisor::publicCancelCallback, this);
    if (require_terrain_health_) {
      terrain_health_sub_ = nh_.subscribe(
          "/terrain/healthy", 1,
          &NavigationSupervisor::terrainHealthCallback, this);
      terrain_health_timer_ = nh_.createWallTimer(
          ros::WallDuration(0.10),
          &NavigationSupervisor::terrainHealthTimerCallback, this);
    }
    internal_server_timer_ = nh_.createWallTimer(
        ros::WallDuration(0.10),
        &NavigationSupervisor::internalServerTimerCallback, this);
    manual_resume_timer_ = nh_.createWallTimer(ros::WallDuration(0.20),
        &NavigationSupervisor::manualResumeCallback, this);

    ready_pub_ =
        nh_.advertise<std_msgs::Bool>("/navigation/ready", 1, true);
    zero_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_nav", 1);
    clear_client_ =
        nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    resume_client_ = nh_.serviceClient<std_srvs::Trigger>(
        "/go2_sdk_bridge_real/resume_after_manual");
    reset_service_ = pnh_.advertiseService(
        "reset", &NavigationSupervisor::resetCallback, this);

    external_action_server_.registerGoalCallback(
        boost::bind(&NavigationSupervisor::externalGoalCallback, this));
    external_action_server_.registerPreemptCallback(
        boost::bind(&NavigationSupervisor::externalPreemptCallback, this));
    external_action_server_.start();
    publishReady(false);
  }

 private:
  void publishReady(bool is_ready) {
    std_msgs::Bool msg;
    msg.data = is_ready;
    ready_pub_.publish(msg);
  }

  void stopInternalGoal() {
    ++action_generation_;
    simple_internal_active_ = false;
    // Address the exact private goal ID. A delayed cancel-all can otherwise
    // cancel a replacement/resumed goal received on a different ROS topic.
    internal_action_client_.cancelGoal();
  }

  void abortExternalGoal(const std::string& reason) {
    if (!external_action_server_.isActive()) {
      return;
    }
    move_base_msgs::MoveBaseResult result;
    external_action_server_.setAborted(result, reason);
  }

  void cancelAndStop(const std::string& reason) {
    if (manual_paused_) manual_resume_blocked_ = true;
    have_retained_goal_ = false;
    resume_attempted_ = true;
    stopInternalGoal();
    abortExternalGoal(reason);
    zero_pub_.publish(geometry_msgs::Twist());
  }

  bool terrainReady() const {
    if (!require_terrain_health_) {
      return true;
    }
    if (!have_terrain_state_ || !terrain_healthy_ || terrain_health_stale_) {
      return false;
    }
    return (ros::WallTime::now() - last_terrain_health_).toSec() <=
        terrain_health_timeout_sec_;
  }

  bool ready() const {
    return localization_ok_ && have_control_state_ && control_enabled_ &&
        terrainReady() && internal_server_connected_;
  }

  bool canRetainManualGoal() const {
    return manual_paused_ && !manual_resume_blocked_ && localization_ok_ && terrainReady() &&
        internal_server_connected_;
  }

  void retainGoal(const move_base_msgs::MoveBaseGoal& goal, bool simple) {
    retained_goal_ = goal;
    retained_is_simple_ = simple;
    have_retained_goal_ = true;
    simple_goal_started_ = simple ? ros::Time::now() : ros::Time();
    resume_attempted_ = false;
  }

  std::string notReadyReason() const {
    return std::string("Navigation is not ready: localization_ok=") +
        (localization_ok_ ? "true" : "false") +
        ", control_enabled=" +
        (control_enabled_ ? "true" : "false") +
        ", terrain_healthy=" +
        (!require_terrain_health_ ? "not-required" :
         (terrainReady() ? "true" : "false-or-stale")) +
        ", move_base_internal=" +
        (internal_server_connected_ ? "connected" : "disconnected") +
        (manual_paused_ ? ". Manual override: target retained until safe resume." :
         ". Restore unhealthy inputs; if control is disabled, run 'run_go2 enable' and publish a fresh goal.");
  }

  void simpleGoalCallback(
      const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (!ready() && !canRetainManualGoal()) {
      const std::string reason = notReadyReason();
      ROS_ERROR_STREAM("Simple goal rejected. " << reason);
      cancelAndStop(reason);
      return;
    }

    if (!internal_action_client_.waitForServer(ros::Duration(0.25))) {
      const std::string reason =
          "Internal move_base action server is unavailable; simple goal rejected";
      ROS_ERROR_STREAM(reason);
      cancelAndStop(reason);
      return;
    }

    // Sending a new action goal lets the action server atomically preempt the
    // previous one. A cancel followed by a separate goal topic has no
    // cross-topic ordering guarantee and can accidentally cancel the new goal.
    if (external_action_server_.isActive()) {
      move_base_msgs::MoveBaseResult result;
      external_action_server_.setPreempted(
          result, "Superseded by a validated simple goal");
    }
    move_base_msgs::MoveBaseGoal internal_goal;
    internal_goal.target_pose = *message;
    retainGoal(internal_goal, true);
    if (manual_paused_) {
      ROS_INFO("Manual override: replacement simple goal retained, not sent to planner");
      return;
    }
    sendInternalGoal(internal_goal, true);
    ROS_INFO_STREAM("Navigation simple goal accepted in frame '"
                    << message->header.frame_id << "'.");
  }

  void externalGoalCallback() {
    if (external_action_server_.isActive()) {
      move_base_msgs::MoveBaseResult result;
      external_action_server_.setPreempted(
          result, "Superseded by a newer action goal");
    }

    const move_base_msgs::MoveBaseGoalConstPtr goal =
        external_action_server_.acceptNewGoal();
    if (!goal) {
      return;
    }
    if (external_action_server_.isPreemptRequested()) {
      have_retained_goal_ = false;
      stopInternalGoal();
      move_base_msgs::MoveBaseResult result;
      external_action_server_.setPreempted(
          result, "Action goal was cancelled before validation completed");
      zero_pub_.publish(geometry_msgs::Twist());
      return;
    }
    if (!ready() && !canRetainManualGoal()) {
      const std::string reason = notReadyReason();
      ROS_ERROR_STREAM("Action goal rejected. " << reason);
      cancelAndStop(reason);
      return;
    }
    if (!internal_action_client_.waitForServer(ros::Duration(0.25))) {
      const std::string reason =
          "Internal move_base action server is unavailable; goal rejected";
      ROS_ERROR_STREAM(reason);
      cancelAndStop(reason);
      return;
    }

    retainGoal(*goal, false);
    if (manual_paused_) {
      ROS_INFO("Manual override: replacement action goal retained and kept active");
      return;
    }
    sendInternalGoal(*goal, false);
    ROS_INFO("Validated action goal forwarded to internal move_base");
  }

  void sendInternalGoal(const move_base_msgs::MoveBaseGoal& goal,
                        bool is_simple_goal) {
    const std::uint64_t generation = ++action_generation_;
    simple_internal_active_ = is_simple_goal;
    internal_action_client_.sendGoal(
        goal,
        boost::bind(&NavigationSupervisor::internalDoneCallback, this,
                    generation, boost::placeholders::_1,
                    boost::placeholders::_2),
        boost::bind(&NavigationSupervisor::internalActiveCallback, this,
                    generation),
        boost::bind(&NavigationSupervisor::internalFeedbackCallback, this,
                    generation, boost::placeholders::_1));
  }

  void externalPreemptCallback() {
    // SimpleActionServer also reports a newer pending goal as a preemption.
    // Do not emit a separate internal cancel in that case: sendGoal() below
    // performs the server-side replacement without a cross-topic race.
    if (external_action_server_.isNewGoalAvailable()) {
      ROS_DEBUG("External action goal will be superseded by a newer goal");
      return;
    }
    if (!external_action_server_.isActive()) {
      return;
    }
    have_retained_goal_ = false;
    stopInternalGoal();
    move_base_msgs::MoveBaseResult result;
    external_action_server_.setPreempted(
        result, "Navigation action cancelled by client");
    zero_pub_.publish(geometry_msgs::Twist());
  }

  void publicCancelCallback(const actionlib_msgs::GoalID::ConstPtr& cancel) {
    if (!(have_retained_goal_ && retained_is_simple_) || !cancel || !cancel->id.empty()) {
      return;
    }
    // actionlib: an empty id with zero stamp cancels all goals; an empty id
    // with a nonzero stamp cancels goals accepted at or before that time.
    if (!cancel->stamp.isZero() && !simple_goal_started_.isZero() &&
        simple_goal_started_ > cancel->stamp) {
      return;
    }
    ROS_WARN("Public move_base cancel stopped the active simple goal");
    have_retained_goal_ = false;
    stopInternalGoal();
    zero_pub_.publish(geometry_msgs::Twist());
  }

  void internalActiveCallback(std::uint64_t generation) {
    if (generation != action_generation_ ||
        !external_action_server_.isActive()) {
      return;
    }
    ROS_DEBUG("Internal move_base accepted the validated action goal");
  }

  void internalFeedbackCallback(
      std::uint64_t generation,
      const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    if (generation != action_generation_ || !feedback ||
        !external_action_server_.isActive()) {
      return;
    }
    external_action_server_.publishFeedback(*feedback);
  }

  void internalDoneCallback(
      std::uint64_t generation,
      const actionlib::SimpleClientGoalState& state,
      const move_base_msgs::MoveBaseResultConstPtr& result_message) {
    if (generation != action_generation_) {
      return;
    }
    simple_internal_active_ = false;
    have_retained_goal_ = false;
    if (!external_action_server_.isActive()) {
      return;
    }
    move_base_msgs::MoveBaseResult result;
    if (result_message) {
      result = *result_message;
    }
    const std::string text =
        std::string("Internal move_base finished: ") + state.toString();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      external_action_server_.setSucceeded(result, text);
    } else if (state == actionlib::SimpleClientGoalState::PREEMPTED ||
               state == actionlib::SimpleClientGoalState::RECALLED) {
      external_action_server_.setPreempted(result, text);
    } else {
      external_action_server_.setAborted(result, text);
    }
  }

  void localizationCallback(const std_msgs::Bool::ConstPtr& message) {
    if (localization_ok_ && !message->data) {
      ROS_ERROR("Localization lost: cancelling navigation goal");
      cancelAndStop("Localization lost");
    }
    localization_ok_ = message->data;
    publishReady(ready());
  }

  void controlEnabledCallback(const std_msgs::Bool::ConstPtr& message) {
    if (have_control_mode_) return;  // Compatibility fallback for older bridges.
    if (have_control_state_ && control_enabled_ && !message->data) {
      ROS_ERROR("GO2 control disabled: cancelling navigation goal");
      cancelAndStop("GO2 control disabled");
    }
    control_enabled_ = message->data;
    have_control_state_ = true;
    publishReady(ready());
  }

  void controlStateCallback(const std_msgs::String::ConstPtr& message) {
    const auto& state = message->data;
    if (state != "enabled" && state != "disabled" &&
        state != "manual_override" && state != "manual_ready") {
      cancelAndStop("Unknown control state");
      control_enabled_ = false;
      publishReady(false);
      return;
    }
    const bool was_enabled = control_enabled_;
    have_control_mode_ = have_control_state_ = true;
    control_enabled_ = state == "enabled";
    if (state == "manual_override" || state == "manual_ready") {
      if (was_enabled) manual_resume_blocked_ = false;
      const bool ready_to_resume = state == "manual_ready";
      if (!manual_paused_) {
        // Cancel only the PRIVATE planner execution so planner patience cannot
        // expire while the operator drives. Keep the user's target/action alive.
        stopInternalGoal();
        zero_pub_.publish(geometry_msgs::Twist());
        ROS_WARN("Remote takeover: navigation target retained; private planner paused");
      }
      if (ready_to_resume != manual_ready_) resume_attempted_ = false;
      manual_paused_ = true;
      manual_ready_ = ready_to_resume;
    } else if (state == "disabled") {
      cancelAndStop("GO2 explicitly disabled or a non-resumable fault occurred");
      manual_paused_ = manual_ready_ = false;
      manual_resume_blocked_ = true;
    } else {
      manual_resume_blocked_ = false;
    }
    // enabled is processed by the timer after queued health/cancel callbacks.
    publishReady(ready() && !manual_paused_);
  }

  void manualResumeCallback(const ros::WallTimerEvent&) {
    if (resume_future_.valid()) {
      if (resume_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return;
      const auto error = resume_future_.get();
      if (!error.empty()) {
        ROS_ERROR_STREAM("Manual resume held: " << error << "; no motion goal sent");
        return;
      }
      ROS_INFO("Manual resume preparation accepted; waiting for enabled state");
    }
    if (!manual_paused_) return;
    if (control_enabled_) {
      if (!ready()) return;
      manual_paused_ = manual_ready_ = false;
      if (have_retained_goal_) {
        sendInternalGoal(retained_goal_, retained_is_simple_);
        ROS_WARN("Remote released: replanning and continuing the retained navigation goal");
      }
      publishReady(ready());
      return;
    }
    if (!manual_ready_ || !have_retained_goal_ || resume_attempted_ ||
        !canRetainManualGoal()) return;
    resume_attempted_ = true;  // One attempt per neutral interval/new target.
    // SDK preparation can take seconds. Keep processing health, cancellation,
    // and replacement goals on the ROS spinner while services are pending.
    resume_future_ = std::async(std::launch::async, [this]() -> std::string {
      std_srvs::Empty clear;
      if (!clear_client_.exists() || !clear_client_.call(clear))
        return "costmap clear failed; target retained";
      std_srvs::Trigger resume;
      if (!resume_client_.exists() || !resume_client_.call(resume))
        return "resume service unavailable; target retained";
      return resume.response.success ? std::string() : resume.response.message;
    });
  }

  void terrainHealthCallback(const std_msgs::Bool::ConstPtr& message) {
    last_terrain_health_ = ros::WallTime::now();
    terrain_health_stale_ = false;
    if (have_terrain_state_ && terrain_healthy_ && !message->data) {
      ROS_ERROR(
          "Terrain obstacle pipeline unhealthy: cancelling navigation goal");
      cancelAndStop("Terrain obstacle pipeline became unhealthy");
    }
    terrain_healthy_ = message->data;
    have_terrain_state_ = true;
    publishReady(ready());
  }

  void terrainHealthTimerCallback(const ros::WallTimerEvent&) {
    if (!require_terrain_health_ || !have_terrain_state_ ||
        !terrain_healthy_ || terrain_health_stale_) {
      return;
    }
    if ((ros::WallTime::now() - last_terrain_health_).toSec() <=
        terrain_health_timeout_sec_) {
      return;
    }
    terrain_health_stale_ = true;
    ROS_ERROR_STREAM("Terrain health heartbeat timed out after "
                     << terrain_health_timeout_sec_
                     << " seconds: cancelling navigation goal");
    cancelAndStop("Terrain health heartbeat timed out");
    publishReady(false);
  }

  void internalServerTimerCallback(const ros::WallTimerEvent&) {
    const bool connected = internal_action_client_.isServerConnected();
    if (have_internal_connection_sample_ &&
        connected == internal_server_connected_) {
      return;
    }
    pnh_.setParam("move_base_internal_connected", connected);
    const bool was_connected = internal_server_connected_;
    internal_server_connected_ = connected;
    have_internal_connection_sample_ = true;
    if (was_connected && !connected) {
      ROS_ERROR(
          "Internal move_base action server disconnected: cancelling goal");
      cancelAndStop("Internal move_base action server disconnected");
      publishReady(false);
      return;
    }
    if (!was_connected && connected) {
      ROS_INFO("Internal move_base action server connected");
    }
    publishReady(ready());
  }

  bool resetCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    cancelAndStop("Navigation reset requested");
    std_srvs::Empty clear;
    const bool available = clear_client_.exists() ||
        clear_client_.waitForExistence(ros::Duration(1.0));
    const bool cleared = available && clear_client_.call(clear);
    response.success = cleared;
    response.message = cleared
        ? "Goal cancelled, zero command sent and costmaps cleared"
        : "Goal cancelled and zero command sent; costmap clear failed";
    if (cleared) {
      ROS_WARN_STREAM(response.message);
    } else {
      ROS_ERROR_STREAM(response.message);
    }
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  MoveBaseActionServer external_action_server_;
  MoveBaseActionClient internal_action_client_;
  ros::Subscriber localization_sub_;
  ros::Subscriber control_enabled_sub_;
  ros::Subscriber control_state_sub_;
  ros::Subscriber simple_goal_sub_;
  ros::Subscriber public_cancel_sub_;
  ros::Subscriber terrain_health_sub_;
  ros::WallTimer terrain_health_timer_;
  ros::WallTimer internal_server_timer_;
  ros::WallTimer manual_resume_timer_;
  ros::Publisher ready_pub_;
  ros::Publisher zero_pub_;
  ros::ServiceClient clear_client_;
  ros::ServiceClient resume_client_;
  ros::ServiceServer reset_service_;
  bool localization_ok_ = false;
  bool control_enabled_ = false;
  bool have_control_state_ = false;
  bool have_control_mode_ = false;
  bool manual_paused_ = false, manual_ready_ = false;
  bool manual_resume_blocked_ = false;
  bool resume_attempted_ = false, have_retained_goal_ = false;
  bool retained_is_simple_ = false;
  move_base_msgs::MoveBaseGoal retained_goal_;
  std::future<std::string> resume_future_;
  bool require_terrain_health_ = false;
  bool terrain_healthy_ = false;
  bool have_terrain_state_ = false;
  bool terrain_health_stale_ = false;
  bool internal_server_connected_ = false;
  bool have_internal_connection_sample_ = false;
  double terrain_health_timeout_sec_ = 0.75;
  ros::WallTime last_terrain_health_;
  std::uint64_t action_generation_ = 0;
  bool simple_internal_active_ = false;
  ros::Time simple_goal_started_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_navigation_supervisor");
  NavigationSupervisor supervisor;
  ros::spin();
  return 0;
}
