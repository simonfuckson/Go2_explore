#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <go2_control/Go2BmsState.h>
#include <go2_control/Go2LowState.h>
#include <go2_control/Go2SportState.h>
#include <go2_control/classic_gait.hpp>
#include <go2_control/manual_control.hpp>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/internal/internal_idl_decl/Request_.hpp>
#include <unitree/common/json/jsonize.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <stdexcept>

namespace
{
constexpr const char* kLowStateTopic = "rt/lowstate";
constexpr const char* kSportStateTopic = "rt/sportmodestate";

template <typename Source, typename Destination>
void copyArray(const Source& source, Destination& destination)
{
  std::copy(source.begin(), source.end(), destination.begin());
}

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value)
{
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template <typename T>
std::string asString(const T& value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}
}  // namespace

class Go2SdkBridgeReal
{
public:
  Go2SdkBridgeReal()
      : nh_(), pnh_("~"), enabled_(false), localization_ok_(false),
        have_cmd_(false)
  {
    pnh_.param<std::string>("command_topic", command_topic_, "/cmd_vel_safe");
    pnh_.param<std::string>("localization_ok_topic", localization_ok_topic_,
                            "/localization/ok");
    pnh_.param<std::string>("network_interface", network_interface_, "eth0");
    pnh_.param<std::string>("gait_mode", gait_mode_, "classic_mcf");
    pnh_.param("allow_motion_mode_switch", allow_motion_mode_switch_, false);
    pnh_.param<std::string>("motion_mode_selector", motion_mode_selector_,
                            "");
    pnh_.param<std::string>("required_motion_mode", required_motion_mode_,
                            "mcf");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param("command_timeout_sec", command_timeout_sec_, 0.50);
    pnh_.param("max_vx", max_vx_, 0.60);
    pnh_.param("max_vy", max_vy_, 0.00);
    pnh_.param("max_wz", max_wz_, 0.80);
    pnh_.param("min_walk_vx", min_walk_vx_, 0.00);
    pnh_.param("min_turn_wz", min_turn_wz_, 0.08);
    pnh_.param("stop_deadband_vx", stop_deadband_vx_, 0.025);
    pnh_.param("stop_deadband_wz", stop_deadband_wz_, 0.01);
    pnh_.param("control_rate_hz", control_rate_hz_, 50.0);
    pnh_.param("telemetry_rate_hz", telemetry_rate_hz_, 50.0);
    pnh_.param("battery_rate_hz", battery_rate_hz_, 5.0);
    pnh_.param("min_enable_battery_percent", min_enable_battery_percent_, 25);
    pnh_.param("motion_mode_settle_sec", motion_mode_settle_sec_, 5.0);
    pnh_.param("motion_response_timeout_sec", motion_response_timeout_sec_,
               6.0);
    pnh_.param("motion_episode_reset_gap_sec",
               motion_episode_reset_gap_sec_, 1.0);
    pnh_.param("min_joint_motion_dq", min_joint_motion_dq_, 0.15);
    pnh_.param("min_yaw_response_wz", min_yaw_response_wz_, 0.05);
    pnh_.param("foot_unload_force_threshold", foot_unload_force_threshold_, 5);
    pnh_.param("require_foot_unload_for_pure_turn",
               require_foot_unload_for_pure_turn_, true);

    if (gait_mode_ != "direct_mcf" && gait_mode_ != "classic_mcf")
    {
      throw std::invalid_argument("gait_mode must be classic_mcf or direct_mcf");
    }

    ROS_WARN("Initializing Unitree ChannelFactory on [%s]",
             network_interface_.c_str());
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

    sport_client_.reset(new unitree::robot::go2::SportClient());
    sport_client_->SetTimeout(1.0f);
    sport_client_->Init();
    ROS_WARN("Unitree sport API client=%s server=%s",
             sport_client_->GetApiVersion().c_str(),
             sport_client_->GetServerApiVersion().c_str());

    motion_switcher_.reset(
        new unitree::robot::b2::MotionSwitcherClient());
    motion_switcher_->SetTimeout(3.0f);
    motion_switcher_->Init();

    command_sub_ = nh_.subscribe(command_topic_, 10,
        &Go2SdkBridgeReal::commandCallback, this);
    localization_sub_ = nh_.subscribe(localization_ok_topic_, 10,
        &Go2SdkBridgeReal::localizationCallback, this);
    enable_service_ = pnh_.advertiseService("enable",
        &Go2SdkBridgeReal::enableCallback, this);
    resume_service_ = pnh_.advertiseService("resume_after_manual",
        &Go2SdkBridgeReal::resumeAfterManual, this);

    low_state_pub_ = nh_.advertise<go2_control::Go2LowState>(
        "/go2/state/low_state", 10);
    sport_state_pub_ = nh_.advertise<go2_control::Go2SportState>(
        "/go2/state/sport_mode", 10);
    bms_state_pub_ = nh_.advertise<go2_control::Go2BmsState>(
        "/go2/state/bms", 10);
    battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>(
        "/go2/battery_state", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/go2/imu", 20);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/go2/joint_states", 20);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "/go2/diagnostics", 10);
    control_enabled_pub_ = nh_.advertise<std_msgs::Bool>(
        "/go2/control/enabled", 1, true);
    control_state_pub_ = nh_.advertise<std_msgs::String>(
        "/go2/control/state", 10, true);

    low_state_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::LowState_>(kLowStateTopic));
    low_state_subscriber_->InitChannel(
        std::bind(&Go2SdkBridgeReal::lowStateCallback, this,
                  std::placeholders::_1), 1);

    sport_state_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>(kSportStateTopic));
    sport_state_subscriber_->InitChannel(
        std::bind(&Go2SdkBridgeReal::sportStateCallback, this,
                  std::placeholders::_1), 1);

    // Observe competing mode/posture commands without replacing the remote's
    // command. ClassicWalk is a latched selection, not a 200 Hz keepalive.
    sport_request_subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_api::msg::dds_::Request_>("rt/api/sport/request"));
    sport_request_subscriber_->InitChannel(
        [this](const void* data) { modeRequestCallback(data, false); }, 10);
    switch_request_subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_api::msg::dds_::Request_>("rt/api/motion_switcher/request"));
    switch_request_subscriber_->InitChannel(
        [this](const void* data) { modeRequestCallback(data, true); }, 10);

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, control_rate_hz_)),
        &Go2SdkBridgeReal::controlCallback, this);
    diagnostics_timer_ = nh_.createTimer(ros::Duration(1.0),
        &Go2SdkBridgeReal::diagnosticsCallback, this);

    stopRobot();
    publishControlEnabled(false);
    ROS_WARN("REAL GO2 SDK bridge started DISABLED; gait=%s, required controller=%s, automatic mode switching=%s.",
             gait_mode_.c_str(), required_motion_mode_.c_str(),
             allow_motion_mode_switch_ ? "enabled" : "disabled");
    ROS_INFO("DDS telemetry: %s and %s", kLowStateTopic, kSportStateTopic);
    ROS_INFO("limits vx=%.3f vy=%.3f wz=%.3f, minimum motion vx=%.3f wz=%.3f",
             max_vx_, max_vy_, max_wz_, min_walk_vx_, min_turn_wz_);
  }

  ~Go2SdkBridgeReal()
  {
    enabled_.store(false);
    { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
    publishControlEnabled(false);
    stopRobot();
    ROS_WARN("GO2 SDK bridge shutdown: StopMove sent.");
  }

private:
  static bool finiteTwist(const geometry_msgs::Twist& cmd)
  {
    return std::isfinite(cmd.linear.x) && std::isfinite(cmd.linear.y) &&
           std::isfinite(cmd.linear.z) && std::isfinite(cmd.angular.x) &&
           std::isfinite(cmd.angular.y) && std::isfinite(cmd.angular.z);
  }

  bool queryMotionMode(std::string& robot_form, std::string& motion_name)
  {
    const int32_t result =
        motion_switcher_->CheckMode(robot_form, motion_name);
    if (result != 0)
    {
      ROS_ERROR("Unitree CheckMode failed: code=%d", result);
      current_motion_mode_ = "check_failed:" + asString(result);
      return false;
    }
    current_motion_mode_ = motion_name.empty() ? "none" : motion_name;
    ROS_WARN("Unitree active motion controller: form=%s mode=%s",
             robot_form.c_str(), current_motion_mode_.c_str());
    return true;
  }

  bool verifyRequiredMotionMode()
  {
    std::string robot_form;
    std::string motion_name;
    if (!queryMotionMode(robot_form, motion_name))
    {
      last_gait_error_ = "MotionSwitcher CheckMode failed";
      return false;
    }
    if (motion_name != required_motion_mode_)
    {
      last_gait_error_ = "active controller is " + current_motion_mode_ +
          ", expected " + required_motion_mode_;
      ROS_ERROR("%s; refusing to switch controllers automatically.",
                last_gait_error_.c_str());
      return false;
    }
    ROS_WARN("Unitree motion controller confirmed [%s].",
             required_motion_mode_.c_str());
    return true;
  }

  bool ensureRequiredMotionMode()
  {
    std::string robot_form;
    std::string motion_name;
    if (!queryMotionMode(robot_form, motion_name))
    {
      last_gait_error_ = "MotionSwitcher CheckMode failed";
      return false;
    }
    if (motion_name == required_motion_mode_)
      return true;

    if (!allow_motion_mode_switch_)
    {
      last_gait_error_ = "active controller is " + current_motion_mode_ +
          ", expected " + required_motion_mode_ +
          "; automatic MotionSwitcher selection is disabled";
      ROS_ERROR("Unitree %s", last_gait_error_.c_str());
      return false;
    }
    if (motion_mode_selector_.empty())
    {
      last_gait_error_ = "motion_mode_selector is empty";
      ROS_ERROR("Unitree %s", last_gait_error_.c_str());
      return false;
    }

    ROS_WARN("Switching Unitree motion controller from [%s] using selector [%s]; expected mode [%s].",
             current_motion_mode_.c_str(), motion_mode_selector_.c_str(),
             required_motion_mode_.c_str());
    const int32_t select_result =
        motion_switcher_->SelectMode(motion_mode_selector_);
    if (select_result != 0)
    {
      last_gait_error_ = "SelectMode(" + motion_mode_selector_ +
          ") failed with SDK code " + asString(select_result);
      ROS_ERROR("Unitree %s", last_gait_error_.c_str());
      return false;
    }

    const ros::WallTime deadline = ros::WallTime::now() +
        ros::WallDuration(std::max(1.0, motion_mode_settle_sec_));
    while (ros::WallTime::now() < deadline)
    {
      ros::WallDuration(0.25).sleep();
      if (queryMotionMode(robot_form, motion_name) &&
          motion_name == required_motion_mode_)
      {
        ROS_WARN("Unitree motion controller switched to [%s].",
                 required_motion_mode_.c_str());
        return true;
      }
    }

    last_gait_error_ = "SelectMode returned success but active controller is " +
        current_motion_mode_ + ", expected " + required_motion_mode_;
    ROS_ERROR("Unitree %s", last_gait_error_.c_str());
    return false;
  }

  bool remoteAllowsArming(bool manual_resume) {
    std::lock_guard<std::mutex> lock(remote_mutex_);
    const double now = ros::WallTime::now().toSec();
    return mode_override_api_.load() == 0 && remote_.neutralAndFresh(now) &&
        (!manual_resume || remote_.canResume(now));
  }

  bool prepareDirectMoveControl(bool preserve_classic_ack = false,
                                bool manual_resume = false)
  {
    last_gait_error_.clear();
    if (!ensureRequiredMotionMode()) return false;
    if (!remoteAllowsArming(manual_resume)) {
      last_gait_error_ = "remote resume permission changed during controller check";
      return false;
    }

    int32_t move_result = 0;
    if (gait_mode_ == "classic_mcf" && !preserve_classic_ack)
    {
      struct PrepareClient {
        Go2SdkBridgeReal& bridge;
        int Move(float x,float y,float z) { return bridge.sport_client_->Move(x,y,z); }
        int ClassicWalk(bool on) {
          if (!on) bridge.expected_classic_reset_request_.store(true);
          const int code = bridge.sport_client_->ClassicWalk(on);
          ROS_WARN("ClassicWalk(%s) preparation reply: SDK=%d", on ? "true" : "false", code);
          return code;
        }
      } client{*this};
      const auto result = go2_control::requestClassicWalk(client,
          [this, manual_resume] { return remoteAllowsArming(manual_resume); },
          [] { ros::WallDuration(0.30).sleep(); });
      expected_classic_reset_request_.store(false);
      move_result = result.zero_result;
      classic_sdk_result_.store(result.classic_result);
      classic_first_result_.store(result.first_classic_result);
      classic_reset_result_.store(result.reset_result);
      classic_reset_attempted_.store(result.reset_attempted);
      if (!result.accepted())
      {
        last_gait_error_ = "ClassicWalk(true) preparation failed: zero SDK=" +
            asString(result.zero_result) + ", first classic SDK=" +
            asString(result.first_classic_result) + ", reset attempted=" +
            asString(result.reset_attempted) + ", reset SDK=" +
            asString(result.reset_result) + ", final classic SDK=" +
            asString(result.classic_result) + ", cancelled=" + asString(result.cancelled);
        ROS_ERROR("%s", last_gait_error_.c_str());
        return false;
      }
      classic_request_accepted_.store(true);
      last_classic_request_wall_.store(ros::WallTime::now().toSec());
      // No Move/StandUp/gait toggles during the selection settling interval.
      ros::WallDuration(0.30).sleep();
      ROS_WARN("ClassicWalk(true), API 2049, acknowledged by MCF. Firmware gait feedback is unavailable; acknowledgement is not a measured gait confirmation.");
    }
    else
    {
      move_result = sport_client_->Move(0.0f, 0.0f, 0.0f);
    }
    last_move_result_.store(move_result);
    if (move_result != 0)
    {
      last_gait_error_ = "Move(0,0,0) failed with SDK code " +
          asString(move_result);
      ROS_ERROR("Unitree %s", last_gait_error_.c_str());
      return false;
    }

    if (!verifyRequiredMotionMode()) return false;

    if (!remoteAllowsArming(manual_resume))
    {
      last_gait_error_ = "remote changed or became stale during arming, API=" +
          asString(mode_override_api_.load());
      return false;
    }
    if (preserve_classic_ack) {
      classic_request_accepted_.store(true);
      ROS_WARN("Joystick-only resume: preserving acknowledged ClassicWalk selection; controller rechecked, no duplicate gait toggle.");
    }
    ROS_WARN("GO2 Move preparation complete: controller=%s gait_policy=%s.",
             current_motion_mode_.c_str(), gait_mode_.c_str());
    return true;
  }

  void modeRequestCallback(const void* data, bool switcher)
  {
    const auto& request =
        *static_cast<const unitree_api::msg::dds_::Request_*>(data);
    const int64_t api = request.header().identity().api_id();
    bool on = false, valid_bool = false;
    if (!switcher && (api == 1027 || api == 2049)) {
      try {
        unitree::common::JsonMap parameter;
        unitree::common::FromJsonString(request.parameter(), parameter);
        const auto found = parameter.find("data");
        if (found != parameter.end()) {
          unitree::common::FromJson(found->second, on); valid_bool = true;
        }
      } catch (...) {}
    }
    const auto override_api = mode_override_api_.load();
    const bool controlled = (enabled_.load() || arming_.load()) &&
        (override_api == 0 || override_api == 1027);
    const auto joystick = switcher ? go2_control::JoystickRequest::kNotJoystick :
        go2_control::joystickRequest(api, valid_bool, on);
    if (joystick != go2_control::JoystickRequest::kNotJoystick) {
      last_joystick_request_.store(valid_bool ? (on ? 1 : 0) : -1);
      ROS_WARN("Observed SwitchJoystick API 1027: data=%s (controller routing, not a gait selection).",
               valid_bool ? (on ? "true" : "false") : "invalid");
      if (joystick == go2_control::JoystickRequest::kTakeover && controlled) {
        std::lock_guard<std::mutex> lock(remote_mutex_);
        remote_.pause(true);
        mode_override_api_.store(1027);
      }
      return;  // A release never disarms an otherwise healthy navigation session.
    }
    if (switcher ? (api != 1002 && api != 1003)
                 : !go2_control::changesSportMode(api)) return;
    if (!switcher && api == 2049)
    {
      if (valid_bool && on) return;
      // Consume exactly one expected off request; any second request aborts.
      if (valid_bool && !on && arming_.load() &&
          expected_classic_reset_request_.exchange(false)) return;
    }
    { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
    classic_request_accepted_.store(false);
    if (!controlled) return;
    mode_override_api_.store(switcher ? -api : api);
  }

  void observeRemote(const std::array<uint8_t,40>& bytes) {
    go2_control::RemoteSample state;
    const bool valid = go2_control::decodeLowStateRemote(bytes, state);
    const auto override_api = mode_override_api_.load();
    const bool controlled = (enabled_.load() || arming_.load()) &&
        (override_api == 0 || override_api == 1027);
    std::lock_guard<std::mutex> lock(remote_mutex_);
    const auto event = remote_.observe(valid ? state.lx : std::numeric_limits<double>::quiet_NaN(),
        state.ly,state.rx,state.ry,state.keys,ros::WallTime::now().toSec(),controlled);
    if (controlled && event != go2_control::RemoteEvent::kNeutral)
      mode_override_api_.store(event == go2_control::RemoteEvent::kManualMotion ? 1027 : -9001);
  }

  bool resumeAfterManual(std_srvs::Trigger::Request&,
                        std_srvs::Trigger::Response& response) {
    {
      std::lock_guard<std::mutex> lock(remote_mutex_);
      if (!remote_.canResume(ros::WallTime::now().toSec())) {
        response.success = false;
        response.message = "Manual resume unavailable: needs prior joystick takeover and fresh centered sticks for 1 second; explicit disable/fault needs run_go2 enable.";
        return true;
      }
    }
    std_srvs::SetBool::Request request;
    std_srvs::SetBool::Response enabled;
    request.data = true;
    setEnabled(request, enabled, true);
    response.success = enabled.success;
    response.message = enabled.message;
    return true;
  }

  double shapeForwardVelocity(double vx) const
  {
    const double magnitude = std::fabs(vx);
    if (magnitude < stop_deadband_vx_) return 0.0;
    return std::copysign(std::min(std::max(magnitude, min_walk_vx_), max_vx_),
                         vx);
  }

  double shapeYawRate(double wz) const
  {
    const double magnitude = std::fabs(wz);
    if (magnitude < stop_deadband_wz_) return 0.0;
    return std::copysign(std::min(std::max(magnitude, min_turn_wz_), max_wz_),
                         wz);
  }

  void localizationCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    const bool previous = localization_ok_.exchange(msg->data);
    if (previous && !msg->data)
    {
      { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
      commanded_motion_active_.store(false);
      last_nonzero_command_wall_.store(0.0);
      commanded_vx_.store(0.0);
      commanded_wz_.store(0.0);
      enabled_.store(false);
      publishControlEnabled(false);
      stopRobot();
      ROS_ERROR("Localization changed from OK to NOT OK; bridge disabled.");
    }
  }

  void commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!finiteTwist(*msg))
    {
      ROS_ERROR_THROTTLE(1.0, "Rejected non-finite velocity command.");
      return;
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_cmd_ = *msg;
    last_cmd_wall_stamp_ = ros::WallTime::now();
    have_cmd_ = true;
  }

  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response)
  {
    return setEnabled(request, response, false);
  }

  bool setEnabled(std_srvs::SetBool::Request& request,
                  std_srvs::SetBool::Response& response, bool manual_resume)
  {
    if (manual_resume) {
      std::lock_guard<std::mutex> lock(remote_mutex_);
      if (!remote_.canResume(ros::WallTime::now().toSec())) {
        response.success = false;
        response.message = "Manual resume permission was revoked before preparation";
        return true;
      }
    }
    // A joystick-only pause does not change the previously acknowledged gait.
    // Any observed gait/posture change clears both acknowledgement and resume
    // permission, so explicit Enable must then reapply ClassicWalk.
    const bool preserve_classic_ack = manual_resume &&
        classic_request_accepted_.load() &&
        (mode_override_api_.load() == 0 || mode_override_api_.load() == 1027);
    arming_.store(false);
    classic_request_accepted_.store(false);
    mode_override_api_.store(0);
    commanded_motion_active_.store(false);
    last_nonzero_command_wall_.store(0.0);
    commanded_vx_.store(0.0);
    commanded_wz_.store(0.0);
    enabled_.store(false);
    publishControlEnabled(false);
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      last_cmd_ = geometry_msgs::Twist();
      last_cmd_wall_stamp_ = ros::WallTime();
      have_cmd_ = false;
    }
    if (!request.data)
    {
      { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
      publishControlEnabled(false);
      stopRobot();
      response.success = true;
      response.message = "REAL GO2 SDK bridge disabled; StopMove sent.";
      ROS_WARN("REAL GO2 motion bridge DISABLED.");
      return true;
    }
    if (!localization_ok_.load())
    {
      { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
      publishControlEnabled(false);
      stopRobot();
      response.success = false;
      response.message = "Cannot enable: localization/ok is false.";
      return true;
    }
    const int battery_soc = battery_soc_.load();
    if (battery_soc < 0 || battery_soc < min_enable_battery_percent_)
    {
      stopRobot();
      response.success = false;
      response.message = "Cannot enable: battery SOC is " +
          asString(battery_soc) + "%, required at least " +
          asString(min_enable_battery_percent_) + "% to test motion.";
      ROS_ERROR("%s", response.message.c_str());
      return true;
    }
    const double now = ros::WallTime::now().toSec();
    if (now - last_low_state_rx_.load() > 0.50 ||
        now - last_sport_state_rx_.load() > 0.50)
    {
      response.success = false;
      response.message = "Cannot enable: current DDS telemetry is unavailable.";
      return true;
    }
    {
      std::lock_guard<std::mutex> lock(remote_mutex_);
      if (!remote_.neutralAndFresh(now)) {
        response.success = false;
        response.message = "Cannot enable: LowState remote data must be valid and fresh, with sticks centered and buttons released for 1 second.";
        return true;
      }
    }
    arming_.store(true);
    const bool prepared = prepareDirectMoveControl(preserve_classic_ack, manual_resume);
    arming_.store(false);
    if (!prepared)
    {
      // Do not counteract a remote Damp/StandDown/controller takeover.
      if (mode_override_api_.load() == 0) stopRobot();
      classic_request_accepted_.store(false);
      response.success = false;
      response.message = "Cannot enable " + gait_mode_ + ": " +
          last_gait_error_ + ".";
      return true;
    }

    commanded_motion_active_.store(false);
    no_step_response_.store(false);
    joint_motion_ema_.store(0.0);
    enabled_.store(true);
    { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
    publishControlEnabled(true);
    response.success = true;
    response.message = "REAL GO2 SDK bridge enabled: " + gait_mode_ +
        "; waiting for a new command. Classic mode uses SDK acknowledgement; firmware gait feedback is not available.";
    ROS_WARN("REAL GO2 motion bridge ENABLED: %s.", gait_mode_.c_str());
    return true;
  }

  void controlCallback(const ros::TimerEvent&)
  {
    if (!sport_client_) return;
    if (!enabled_.load()) {
      publishControlEnabled(false);  // Advance manual_override -> manual_ready.
      return;
    }
    if (mode_override_api_.load() == 1027) {
      enabled_.store(false);
      publishControlEnabled(false);
      commanded_motion_active_.store(false);
      commanded_vx_.store(0.0); commanded_wz_.store(0.0);
      ROS_WARN("Manual joystick takeover: autonomous output paused; supervisor retains the goal and replans after sticks remain centered for 1 second.");
      return;  // Do not fight the operator with zero/StopMove during manual driving.
    }
    if (mode_override_api_.load() != 0 ||
        (gait_mode_ == "classic_mcf" && !classic_request_accepted_.load()))
    {
      { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
      enabled_.store(false);
      publishControlEnabled(false);
      commanded_motion_active_.store(false);
      commanded_vx_.store(0.0);
      commanded_wz_.store(0.0);
      last_gait_error_ = "classic policy invalidated by external mode/posture API " +
          asString(mode_override_api_.load()) + "; manual Enable required";
      // Cease publishing so a manual Stop/Damp/StandDown is not overwritten.
      ROS_ERROR("%s", last_gait_error_.c_str());
      return;
    }
    if (!localization_ok_.load())
    {
      { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
      commanded_motion_active_.store(false);
      commanded_vx_.store(0.0);
      commanded_wz_.store(0.0);
      enabled_.store(false);
      publishControlEnabled(false);
      stopRobot();
      ROS_ERROR_THROTTLE(1.0, "Localization lost. Bridge disabled.");
      return;
    }

    geometry_msgs::Twist cmd;
    ros::WallTime command_stamp;
    bool have_command = false;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      have_command = have_cmd_;
      cmd = last_cmd_;
      command_stamp = last_cmd_wall_stamp_;
    }

    if (!have_command)
    {
      holdZeroVelocity();
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    const double age = (now - command_stamp).toSec();
    if (age > command_timeout_sec_)
    {
      commanded_motion_active_.store(false);
      commanded_vx_.store(0.0);
      commanded_wz_.store(0.0);
      holdZeroVelocity();
      ROS_WARN_THROTTLE(1.0, "Velocity command timeout: %.3f sec", age);
      return;
    }

    const double vx = shapeForwardVelocity(cmd.linear.x);
    const double vy = 0.0;
    const double wz = shapeYawRate(cmd.angular.z);
    if (std::fabs(vx) < 1e-6 && std::fabs(wz) < 1e-6)
    {
      commanded_motion_active_.store(false);
      commanded_vx_.store(0.0);
      commanded_wz_.store(0.0);
      holdZeroVelocity();
      return;
    }

    commanded_vx_.store(vx);
    commanded_wz_.store(wz);
    const double previous_nonzero_wall =
        last_nonzero_command_wall_.exchange(now.toSec());
    const bool new_motion_episode = previous_nonzero_wall <= 0.0 ||
        now.toSec() - previous_nonzero_wall >= motion_episode_reset_gap_sec_;
    if (!commanded_motion_active_.exchange(true) && new_motion_episode)
    {
      motion_command_start_.store(now.toSec());
      last_motion_response_wall_.store(now.toSec());
      last_foot_unload_wall_.store(0.0);
      joint_motion_ema_.store(0.0);
      no_step_response_.store(false);
    }
    else if (!new_motion_episode)
    {
      ROS_DEBUG_THROTTLE(
          1.0,
          "Motion resumed after a short zero-command gap; retaining response watchdog state");
    }

    const double motion_age = now.toSec() - motion_command_start_.load();
    const double no_response_age =
        now.toSec() - last_motion_response_wall_.load();
    if (no_response_age >= motion_response_timeout_sec_)
    {
      { std::lock_guard<std::mutex> lock(remote_mutex_); remote_.cancelResume(); }
      no_step_response_.store(true);
      commanded_motion_active_.store(false);
      last_nonzero_command_wall_.store(0.0);
      enabled_.store(false);
      publishControlEnabled(false);
      stopRobot();
      ROS_ERROR("GO2 command vx=%.3f wz=%.3f ran for %.1f sec without physical response for %.1f sec; bridge disabled.",
                vx, wz, motion_age, no_response_age);
      return;
    }

    const int32_t move_result = sport_client_->Move(vx, vy, wz);
    last_move_result_.store(move_result);
    if (move_result != 0)
      ROS_ERROR_THROTTLE(1.0, "Unitree Move failed with SDK code %d", move_result);
    ROS_INFO_THROTTLE(0.5,
        "GO2 MOVE raw=(%.3f %.3f %.3f) shaped=(%.3f %.3f %.3f)",
        cmd.linear.x, cmd.linear.y, cmd.angular.z, vx, vy, wz);
  }

  void fillBmsMessage(const unitree_go::msg::dds_::BmsState_& source,
                      const std_msgs::Header& header,
                      go2_control::Go2BmsState& target) const
  {
    target.header = header;
    target.version_high = source.version_high();
    target.version_low = source.version_low();
    target.status = source.status();
    target.soc = source.soc();
    target.current_ma = source.current();
    target.cycle = source.cycle();
    copyArray(source.bq_ntc(), target.bq_ntc);
    copyArray(source.mcu_ntc(), target.mcu_ntc);
    copyArray(source.cell_vol(), target.cell_voltage_mv);
  }

  void lowStateCallback(const void* message)
  {
    const auto& state =
        *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
    observeRemote(state.wireless_remote());
    const double wall_now = ros::WallTime::now().toSec();
    last_low_state_rx_.store(wall_now);
    battery_soc_.store(static_cast<int>(state.bms_state().soc()));
    battery_voltage_.store(state.power_v());
    battery_current_.store(state.power_a());
    const double imu_yaw_rate =
        static_cast<double>(state.imu_state().gyroscope()[2]);
    imu_yaw_rate_.store(imu_yaw_rate);

    double max_abs_joint_dq = 0.0;
    for (std::size_t i = 0; i < 12; ++i)
      max_abs_joint_dq = std::max(
          max_abs_joint_dq,
          std::fabs(static_cast<double>(state.motor_state()[i].dq())));
    max_joint_dq_.store(max_abs_joint_dq);
    const int16_t min_foot_force = *std::min_element(
        state.foot_force().begin(), state.foot_force().end());
    min_foot_force_.store(static_cast<int>(min_foot_force));
    if (commanded_motion_active_.load())
    {
      const double previous = joint_motion_ema_.load();
      joint_motion_ema_.store(0.9 * previous + 0.1 * max_abs_joint_dq);
      const bool foot_unloaded =
          min_foot_force <= foot_unload_force_threshold_;
      if (foot_unloaded)
        last_foot_unload_wall_.store(wall_now);
      const bool pure_turn =
          std::fabs(commanded_vx_.load()) < stop_deadband_vx_ &&
          std::fabs(commanded_wz_.load()) >= stop_deadband_wz_;
      const bool yaw_responded =
          std::fabs(imu_yaw_rate) >= min_yaw_response_wz_;
      const bool responded = pure_turn
          ? (yaw_responded &&
             (!require_foot_unload_for_pure_turn_ || foot_unloaded))
          : (max_abs_joint_dq >= min_joint_motion_dq_ ||
             foot_unloaded);
      if (responded)
      {
        last_motion_response_wall_.store(wall_now);
        no_step_response_.store(false);
      }
    }

    const double period = 1.0 / std::max(1.0, telemetry_rate_hz_);
    if (wall_now - last_low_state_publish_.load() < period) return;
    last_low_state_publish_.store(wall_now);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = base_frame_;

    go2_control::Go2LowState output;
    output.header = header;
    copyArray(state.head(), output.head);
    output.level_flag = state.level_flag();
    output.frame_reserve = state.frame_reserve();
    copyArray(state.sn(), output.serial_number);
    copyArray(state.version(), output.version);
    output.bandwidth = state.bandwidth();
    copyArray(state.imu_state().quaternion(), output.imu_quaternion_wxyz);
    copyArray(state.imu_state().gyroscope(), output.imu_gyroscope);
    copyArray(state.imu_state().accelerometer(), output.imu_accelerometer);
    copyArray(state.imu_state().rpy(), output.imu_rpy);
    output.imu_temperature = state.imu_state().temperature();
    for (std::size_t i = 0; i < state.motor_state().size(); ++i)
    {
      const auto& source = state.motor_state()[i];
      auto& target = output.motor_state[i];
      target.mode = source.mode();
      target.q = source.q();
      target.dq = source.dq();
      target.ddq = source.ddq();
      target.tau_est = source.tau_est();
      target.q_raw = source.q_raw();
      target.dq_raw = source.dq_raw();
      target.ddq_raw = source.ddq_raw();
      target.temperature = source.temperature();
      target.lost = source.lost();
      copyArray(source.reserve(), target.reserve);
    }
    fillBmsMessage(state.bms_state(), header, output.bms_state);
    copyArray(state.foot_force(), output.foot_force);
    copyArray(state.foot_force_est(), output.foot_force_est);
    output.tick = state.tick();
    copyArray(state.wireless_remote(), output.wireless_remote);
    output.bit_flag = state.bit_flag();
    output.adc_reel = state.adc_reel();
    output.temperature_ntc1 = state.temperature_ntc1();
    output.temperature_ntc2 = state.temperature_ntc2();
    output.power_voltage = state.power_v();
    output.power_current = state.power_a();
    copyArray(state.fan_frequency(), output.fan_frequency);
    output.reserve = state.reserve();
    output.crc = state.crc();
    low_state_pub_.publish(output);

    sensor_msgs::Imu imu;
    imu.header = header;
    const auto& q = state.imu_state().quaternion();  // SDK: w,x,y,z
    imu.orientation.w = q[0];
    imu.orientation.x = q[1];
    imu.orientation.y = q[2];
    imu.orientation.z = q[3];
    imu.angular_velocity.x = state.imu_state().gyroscope()[0];
    imu.angular_velocity.y = state.imu_state().gyroscope()[1];
    imu.angular_velocity.z = state.imu_state().gyroscope()[2];
    imu.linear_acceleration.x = state.imu_state().accelerometer()[0];
    imu.linear_acceleration.y = state.imu_state().accelerometer()[1];
    imu.linear_acceleration.z = state.imu_state().accelerometer()[2];
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_.publish(imu);

    static const std::array<std::string, 12> joint_names = {{
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}};
    sensor_msgs::JointState joints;
    joints.header = header;
    joints.name.assign(joint_names.begin(), joint_names.end());
    joints.position.resize(12);
    joints.velocity.resize(12);
    joints.effort.resize(12);
    for (std::size_t i = 0; i < 12; ++i)
    {
      joints.position[i] = state.motor_state()[i].q();
      joints.velocity[i] = state.motor_state()[i].dq();
      joints.effort[i] = state.motor_state()[i].tau_est();
    }
    joint_state_pub_.publish(joints);

    const double battery_period = 1.0 / std::max(0.2, battery_rate_hz_);
    if (wall_now - last_battery_publish_.load() >= battery_period)
    {
      last_battery_publish_.store(wall_now);
      bms_state_pub_.publish(output.bms_state);
      sensor_msgs::BatteryState battery;
      battery.header = header;
      battery.voltage = state.power_v();
      battery.current = static_cast<float>(state.bms_state().current()) / 1000.0f;
      battery.temperature = 0.5f *
          (static_cast<float>(state.bms_state().bq_ntc()[0]) +
           static_cast<float>(state.bms_state().bq_ntc()[1]));
      battery.charge = std::numeric_limits<float>::quiet_NaN();
      battery.capacity = std::numeric_limits<float>::quiet_NaN();
      battery.design_capacity = std::numeric_limits<float>::quiet_NaN();
      battery.percentage = static_cast<float>(state.bms_state().soc()) / 100.0f;
      battery.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
      battery.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
      battery.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      battery.present = true;
      // The DDS schema reserves 15 cells, while this GO2 pack reports eight.
      // Preserve all 15 raw entries in /go2/state/bms, but omit zero-valued
      // reserved entries from the standard BatteryState topic.
      for (const uint16_t millivolts : state.bms_state().cell_vol())
      {
        if (millivolts > 0)
          battery.cell_voltage.push_back(millivolts / 1000.0f);
      }
      battery.location = "GO2 chassis battery";
      battery_pub_.publish(battery);
    }
  }

  void sportStateCallback(const void* message)
  {
    const auto& state =
        *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
    const double wall_now = ros::WallTime::now().toSec();
    last_sport_state_rx_.store(wall_now);
    sport_mode_.store(static_cast<int>(state.mode()));
    gait_type_.store(static_cast<int>(state.gait_type()));
    foot_raise_height_.store(state.foot_raise_height());
    sport_state_error_code_.store(static_cast<int>(state.error_code()));

    const double period = 1.0 / std::max(1.0, telemetry_rate_hz_);
    if (wall_now - last_sport_state_publish_.load() < period) return;
    last_sport_state_publish_.store(wall_now);

    go2_control::Go2SportState output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = base_frame_;
    output.dds_stamp_sec = state.stamp().sec();
    output.dds_stamp_nanosec = state.stamp().nanosec();
    output.error_code = state.error_code();
    copyArray(state.imu_state().quaternion(), output.imu_quaternion_wxyz);
    copyArray(state.imu_state().gyroscope(), output.imu_gyroscope);
    copyArray(state.imu_state().accelerometer(), output.imu_accelerometer);
    copyArray(state.imu_state().rpy(), output.imu_rpy);
    output.imu_temperature = state.imu_state().temperature();
    output.mode = state.mode();
    output.progress = state.progress();
    output.gait_type = state.gait_type();
    output.foot_raise_height = state.foot_raise_height();
    copyArray(state.position(), output.position);
    output.body_height = state.body_height();
    copyArray(state.velocity(), output.velocity);
    output.yaw_speed = state.yaw_speed();
    copyArray(state.range_obstacle(), output.range_obstacle);
    copyArray(state.foot_force(), output.foot_force);
    copyArray(state.foot_position_body(), output.foot_position_body);
    copyArray(state.foot_speed_body(), output.foot_speed_body);
    for (std::size_t i = 0; i < state.path_point().size(); ++i)
    {
      output.path_point[i].time_from_start = state.path_point()[i].t_from_start();
      output.path_point[i].x = state.path_point()[i].x();
      output.path_point[i].y = state.path_point()[i].y();
      output.path_point[i].yaw = state.path_point()[i].yaw();
      output.path_point[i].vx = state.path_point()[i].vx();
      output.path_point[i].vy = state.path_point()[i].vy();
      output.path_point[i].vyaw = state.path_point()[i].vyaw();
    }
    sport_state_pub_.publish(output);
  }

  void diagnosticsCallback(const ros::TimerEvent&)
  {
    const double now = ros::WallTime::now().toSec();
    const double low_age = last_low_state_rx_.load() > 0.0
        ? now - last_low_state_rx_.load() : std::numeric_limits<double>::infinity();
    const double sport_age = last_sport_state_rx_.load() > 0.0
        ? now - last_sport_state_rx_.load() : std::numeric_limits<double>::infinity();
    const double response_stamp = last_motion_response_wall_.load();
    const double response_age = commanded_motion_active_.load() && response_stamp > 0.0
        ? now - response_stamp : 0.0;
    const double foot_unload_stamp = last_foot_unload_wall_.load();
    const double foot_unload_age = commanded_motion_active_.load() &&
        foot_unload_stamp > 0.0 ? now - foot_unload_stamp : -1.0;
    const double last_nonzero_stamp = last_nonzero_command_wall_.load();
    const double last_nonzero_age = last_nonzero_stamp > 0.0
        ? now - last_nonzero_stamp : -1.0;

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "GO2 SDK bridge";
    status.hardware_id = "unitree_go2";
    if (no_step_response_.load())
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "Move commands accepted but no stepping response; bridge disabled";
    }
    else if (low_age > 1.0 || sport_age > 1.0)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "Unitree DDS telemetry stale or missing";
    }
    else if (battery_soc_.load() < min_enable_battery_percent_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "GO2 battery low";
    }
    else
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = enabled_.load() ? "telemetry OK, motion enabled"
                                       : "telemetry OK, motion disabled";
    }
    status.values.push_back(keyValue("motion_enabled", enabled_.load() ? "true" : "false"));
    status.values.push_back(keyValue("localization_ok", localization_ok_.load() ? "true" : "false"));
    status.values.push_back(keyValue("gait_mode", gait_mode_));
    status.values.push_back(keyValue("allow_motion_mode_switch",
        allow_motion_mode_switch_ ? "true" : "false"));
    status.values.push_back(keyValue("motion_mode_selector", motion_mode_selector_));
    status.values.push_back(keyValue("required_motion_mode", required_motion_mode_));
    status.values.push_back(keyValue("active_motion_mode", current_motion_mode_));
    status.values.push_back(keyValue("last_gait_error", last_gait_error_));
    status.values.push_back(keyValue("classic_api_id", "2049"));
    status.values.push_back(keyValue("classic_sdk_result", asString(classic_sdk_result_.load())));
    status.values.push_back(keyValue("classic_first_sdk_result", asString(classic_first_result_.load())));
    status.values.push_back(keyValue("classic_reset_attempted", classic_reset_attempted_.load() ? "true" : "false"));
    status.values.push_back(keyValue("classic_reset_sdk_result", asString(classic_reset_result_.load())));
    status.values.push_back(keyValue("last_switch_joystick_data", asString(last_joystick_request_.load())));
    {
      std::lock_guard<std::mutex> lock(remote_mutex_);
      status.values.push_back(keyValue("manual_resume_pending", remote_.resumable ? "true" : "false"));
      status.values.push_back(keyValue("remote_source", "rt/lowstate.wireless_remote"));
      status.values.push_back(keyValue("manual_resume_ready", remote_.canResume(now) ? "true" : "false"));
      status.values.push_back(keyValue("remote_axis_max", asString(remote_.max_axis)));
      status.values.push_back(keyValue("remote_buttons", asString(remote_.keys)));
      status.values.push_back(keyValue("remote_age_sec", remote_.last_sample < 0.0 ? "unavailable" : asString(now-remote_.last_sample)));
    }
    status.values.push_back(keyValue("classic_request_accepted", classic_request_accepted_.load() ? "true" : "false"));
    status.values.push_back(keyValue("classic_feedback_verified", "false"));
    status.values.push_back(keyValue("classic_feedback_note", "MCF mode/gait fields and GetState do not confirm gait on this firmware"));
    status.values.push_back(keyValue("last_classic_request_age_sec", last_classic_request_wall_.load() > 0.0 ? asString(now-last_classic_request_wall_.load()) : "not_requested"));
    status.values.push_back(keyValue("mode_override_api", asString(mode_override_api_.load())));
    status.values.push_back(keyValue("last_move_sdk_result", asString(last_move_result_.load())));
    status.values.push_back(keyValue("control_rate_hz", asString(control_rate_hz_)));
    status.values.push_back(keyValue("motion_command_active",
        commanded_motion_active_.load() ? "true" : "false"));
    status.values.push_back(keyValue("commanded_vx_m_s", asString(commanded_vx_.load())));
    status.values.push_back(keyValue("commanded_wz_rad_s", asString(commanded_wz_.load())));
    status.values.push_back(keyValue("imu_yaw_rate_rad_s", asString(imu_yaw_rate_.load())));
    status.values.push_back(keyValue("min_yaw_response_wz", asString(min_yaw_response_wz_)));
    status.values.push_back(keyValue("require_foot_unload_for_pure_turn",
        require_foot_unload_for_pure_turn_ ? "true" : "false"));
    status.values.push_back(keyValue("min_foot_force", asString(min_foot_force_.load())));
    status.values.push_back(keyValue("foot_unload_force_threshold",
        asString(foot_unload_force_threshold_)));
    status.values.push_back(keyValue("last_foot_unload_age_sec",
        asString(foot_unload_age)));
    status.values.push_back(keyValue("last_motion_response_age_sec", asString(response_age)));
    status.values.push_back(keyValue("motion_response_timeout_sec", asString(motion_response_timeout_sec_)));
    status.values.push_back(keyValue("motion_episode_reset_gap_sec",
        asString(motion_episode_reset_gap_sec_)));
    status.values.push_back(keyValue("last_nonzero_command_age_sec",
        asString(last_nonzero_age)));
    status.values.push_back(keyValue("no_step_response", no_step_response_.load() ? "true" : "false"));
    status.values.push_back(keyValue("max_joint_dq_rad_s", asString(max_joint_dq_.load())));
    status.values.push_back(keyValue("joint_motion_ema_rad_s", asString(joint_motion_ema_.load())));
    status.values.push_back(keyValue("sport_state_error_code", asString(sport_state_error_code_.load())));
    status.values.push_back(keyValue("sport_mode", asString(sport_mode_.load())));
    status.values.push_back(keyValue("gait_type", asString(gait_type_.load())));
    status.values.push_back(keyValue("foot_raise_height_m", asString(foot_raise_height_.load())));
    status.values.push_back(keyValue("battery_soc_percent", asString(battery_soc_.load())));
    status.values.push_back(keyValue("min_enable_battery_percent", asString(min_enable_battery_percent_)));
    status.values.push_back(keyValue("power_voltage_v", asString(battery_voltage_.load())));
    status.values.push_back(keyValue("power_current_a", asString(battery_current_.load())));
    status.values.push_back(keyValue("low_state_age_sec", asString(low_age)));
    status.values.push_back(keyValue("sport_state_age_sec", asString(sport_age)));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  void stopRobot()
  {
    if (sport_client_) sport_client_->StopMove();
  }

  void publishControlEnabled(bool enabled)
  {
    if (!control_enabled_pub_) return;
    std_msgs::String state;
    state.data = enabled ? "enabled" : "disabled";
    if (!enabled) {
      std::lock_guard<std::mutex> lock(remote_mutex_);
      if (remote_.resumable)
        state.data = remote_.canResume(ros::WallTime::now().toSec())
            ? "manual_ready" : "manual_override";
    }
    // One ordered topic carries both enabled state and its reason. A separate
    // bool/reason pair has no ROS cross-topic ordering guarantee.
    if (state.data != last_published_state_) {
      last_published_state_ = state.data;
      control_state_pub_.publish(state);
    }
    // Do not enqueue another stale false before a successful resume's true.
    if (have_published_control_ && last_published_control_ == enabled) return;
    have_published_control_ = true;
    last_published_control_ = enabled;
    std_msgs::Bool msg;
    msg.data = enabled;
    control_enabled_pub_.publish(msg);
  }

  void holdZeroVelocity()
  {
    if (!sport_client_) return;
    last_move_result_.store(sport_client_->Move(0.0, 0.0, 0.0));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber command_sub_;
  ros::Subscriber localization_sub_;
  ros::ServiceServer enable_service_;
  ros::ServiceServer resume_service_;
  ros::Publisher low_state_pub_;
  ros::Publisher sport_state_pub_;
  ros::Publisher bms_state_pub_;
  ros::Publisher battery_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher control_enabled_pub_;
  ros::Publisher control_state_pub_;
  std::string last_published_state_;
  ros::Timer control_timer_;
  ros::Timer diagnostics_timer_;

  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
  std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      low_state_subscriber_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
      sport_state_subscriber_;
  unitree::robot::ChannelSubscriberPtr<unitree_api::msg::dds_::Request_>
      sport_request_subscriber_, switch_request_subscriber_;
  std::atomic<bool> arming_{false};
  std::atomic<bool> classic_request_accepted_{false};
  std::atomic<int32_t> classic_sdk_result_{-1};
  std::atomic<int32_t> classic_first_result_{-1}, classic_reset_result_{-1};
  std::atomic<bool> classic_reset_attempted_{false};
  std::atomic<bool> expected_classic_reset_request_{false};
  std::atomic<int> last_joystick_request_{-1};
  std::mutex remote_mutex_;
  go2_control::ManualControlState remote_;
  bool have_published_control_ = false, last_published_control_ = false;
  std::atomic<int64_t> mode_override_api_{0};
  std::atomic<double> last_classic_request_wall_{0.0};

  std::atomic<bool> enabled_;
  std::atomic<bool> localization_ok_;
  std::mutex cmd_mutex_;
  geometry_msgs::Twist last_cmd_;
  ros::WallTime last_cmd_wall_stamp_;
  bool have_cmd_;

  std::atomic<double> last_low_state_rx_{0.0};
  std::atomic<double> last_sport_state_rx_{0.0};
  std::atomic<double> last_low_state_publish_{0.0};
  std::atomic<double> last_sport_state_publish_{0.0};
  std::atomic<double> last_battery_publish_{0.0};
  std::atomic<int> battery_soc_{-1};
  std::atomic<double> battery_voltage_{0.0};
  std::atomic<double> battery_current_{0.0};
  std::atomic<int> sport_mode_{-1};
  std::atomic<int> gait_type_{-1};
  std::atomic<double> foot_raise_height_{0.0};
  std::atomic<int> sport_state_error_code_{-1};
  std::atomic<int> last_move_result_{0};
  std::atomic<bool> commanded_motion_active_{false};
  std::atomic<bool> no_step_response_{false};
  std::atomic<double> motion_command_start_{0.0};
  std::atomic<double> last_motion_response_wall_{0.0};
  std::atomic<double> last_nonzero_command_wall_{0.0};
  std::atomic<double> commanded_vx_{0.0};
  std::atomic<double> commanded_wz_{0.0};
  std::atomic<double> imu_yaw_rate_{0.0};
  std::atomic<double> last_foot_unload_wall_{0.0};
  std::atomic<int> min_foot_force_{-1};
  std::atomic<double> max_joint_dq_{0.0};
  std::atomic<double> joint_motion_ema_{0.0};

  std::string command_topic_;
  std::string localization_ok_topic_;
  std::string network_interface_;
  std::string gait_mode_;
  bool allow_motion_mode_switch_;
  std::string motion_mode_selector_;
  std::string required_motion_mode_;
  std::string current_motion_mode_ = "not_checked";
  std::string last_gait_error_ = "not_attempted";
  std::string base_frame_;
  double command_timeout_sec_;
  double max_vx_;
  double max_vy_;
  double max_wz_;
  double min_walk_vx_;
  double min_turn_wz_;
  double stop_deadband_vx_;
  double stop_deadband_wz_;
  double control_rate_hz_;
  double telemetry_rate_hz_;
  double battery_rate_hz_;
  int min_enable_battery_percent_;
  double motion_mode_settle_sec_;
  double motion_response_timeout_sec_;
  double motion_episode_reset_gap_sec_;
  double min_joint_motion_dq_;
  double min_yaw_response_wz_;
  int foot_unload_force_threshold_;
  bool require_foot_unload_for_pure_turn_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go2_sdk_bridge_real");
  ROS_WARN("Starting REAL Unitree GO2 SDK bridge.");
  Go2SdkBridgeReal node;
  ros::spin();
  return 0;
}
