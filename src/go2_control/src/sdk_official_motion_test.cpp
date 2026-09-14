#include <ros/master.h>
#include <ros/ros.h>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr const char* kExecuteToken = "--execute-straight-test";
constexpr const char* kExecuteStandingToken = "--execute-standing-test";
constexpr float kTestVx = 0.30f;
constexpr int kPeriodMs = 5;
constexpr int kCommandCount = 200;

class MotionTelemetry
{
public:
  void reset()
  {
    collecting_.store(true);
    max_joint_dq_.store(0.0);
    min_foot_force_.store(32767);
    saw_foot_unload_.store(false);
    samples_.store(0);
    remote_valid_.store(false);
    remote_button_activity_.store(false);
    max_remote_axis_.store(0.0);
  }

  void stop() { collecting_.store(false); }

  void callback(const void* message)
  {
    if (!collecting_.load()) return;
    const auto& state =
        *static_cast<const unitree_go::msg::dds_::LowState_*>(message);

    double max_dq = 0.0;
    for (std::size_t i = 0; i < 12; ++i)
      max_dq = std::max(max_dq,
          std::fabs(static_cast<double>(state.motor_state()[i].dq())));
    double previous = max_joint_dq_.load();
    while (max_dq > previous &&
           !max_joint_dq_.compare_exchange_weak(previous, max_dq)) {}

    const int min_force = *std::min_element(
        state.foot_force().begin(), state.foot_force().end());
    int previous_force = min_foot_force_.load();
    while (min_force < previous_force &&
           !min_foot_force_.compare_exchange_weak(previous_force, min_force)) {}
    if (min_force <= 5) saw_foot_unload_.store(true);

    const auto& remote = state.wireless_remote();
    remote_valid_.store(remote[0] == 0x55 && remote[1] == 0x51);
    uint16_t buttons = 0;
    std::memcpy(&buttons, remote.data() + 2, sizeof(buttons));
    if (buttons != 0) remote_button_activity_.store(true);
    for (std::size_t offset : {4U, 8U, 12U, 16U, 20U})
    {
      float axis = 0.0f;
      std::memcpy(&axis, remote.data() + offset, sizeof(axis));
      double previous_axis = max_remote_axis_.load();
      const double magnitude = std::fabs(static_cast<double>(axis));
      while (magnitude > previous_axis &&
             !max_remote_axis_.compare_exchange_weak(previous_axis, magnitude)) {}
    }
    samples_.fetch_add(1);
  }

  double maxJointDq() const { return max_joint_dq_.load(); }
  int minFootForce() const { return min_foot_force_.load(); }
  bool sawFootUnload() const { return saw_foot_unload_.load(); }
  int samples() const { return samples_.load(); }
  bool remoteValid() const { return remote_valid_.load(); }
  bool remoteButtonActivity() const { return remote_button_activity_.load(); }
  double maxRemoteAxis() const { return max_remote_axis_.load(); }

private:
  std::atomic<bool> collecting_{false};
  std::atomic<double> max_joint_dq_{0.0};
  std::atomic<int> min_foot_force_{32767};
  std::atomic<bool> saw_foot_unload_{false};
  std::atomic<int> samples_{0};
  std::atomic<bool> remote_valid_{false};
  std::atomic<bool> remote_button_activity_{false};
  std::atomic<double> max_remote_axis_{0.0};
};

class SportTelemetry
{
public:
  void callback(const void* message)
  {
    const auto& state =
        *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
    error_code_.store(state.error_code());
    mode_.store(state.mode());
    gait_type_.store(state.gait_type());
    foot_raise_height_.store(state.foot_raise_height());
    const double speed = std::hypot(
        static_cast<double>(state.velocity()[0]),
        static_cast<double>(state.velocity()[1]));
    double previous = max_reported_speed_.load();
    while (speed > previous &&
           !max_reported_speed_.compare_exchange_weak(previous, speed)) {}
    samples_.fetch_add(1);
  }

  uint32_t errorCode() const { return error_code_.load(); }
  int mode() const { return mode_.load(); }
  int gaitType() const { return gait_type_.load(); }
  double footRaiseHeight() const { return foot_raise_height_.load(); }
  double maxReportedSpeed() const { return max_reported_speed_.load(); }
  int samples() const { return samples_.load(); }

private:
  std::atomic<uint32_t> error_code_{0};
  std::atomic<int> mode_{-1};
  std::atomic<int> gait_type_{-1};
  std::atomic<double> foot_raise_height_{0.0};
  std::atomic<double> max_reported_speed_{0.0};
  std::atomic<int> samples_{0};
};

class StopGuard
{
public:
  explicit StopGuard(unitree::robot::go2::SportClient& client)
      : client_(client) {}
  ~StopGuard()
  {
    client_.Move(0.0f, 0.0f, 0.0f);
    client_.StopMove();
  }

private:
  unitree::robot::go2::SportClient& client_;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go2_sdk_official_motion_test",
            ros::init_options::NoSigintHandler);
  if (argc != 3 ||
      (std::string(argv[2]) != kExecuteToken &&
       std::string(argv[2]) != kExecuteStandingToken))
  {
    std::cerr << "Usage: rosrun go2_control go2_sdk_official_motion_test_node "
              << "<network_interface> " << kExecuteToken << std::endl;
    std::cerr << "This command makes the robot walk forward at 0.30 m/s for "
              << "one second. Stop the navigation launch first." << std::endl;
    return 2;
  }
  const bool call_stand_up = std::string(argv[2]) == kExecuteToken;

  if (ros::master::check())
  {
    std::vector<std::string> nodes;
    if (ros::master::getNodes(nodes) &&
        std::find(nodes.begin(), nodes.end(), "/go2_sdk_bridge_real") !=
            nodes.end())
    {
      std::cerr << "Refusing test: /go2_sdk_bridge_real is running. Stop the "
                << "navigation launch first." << std::endl;
      return 3;
    }
  }

  const std::string network_interface = argv[1];
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(1.0f);
  sport_client.Init();
  StopGuard stop_guard(sport_client);

  std::cout << "SportClient API client=" << sport_client.GetApiVersion()
            << " server=" << sport_client.GetServerApiVersion() << std::endl;
  if (sport_client.GetApiVersion() != sport_client.GetServerApiVersion())
  {
    std::cerr << "Refusing test: SportClient API versions do not match."
              << std::endl;
    return 4;
  }

  MotionTelemetry telemetry;
  auto low_state_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<
      unitree_go::msg::dds_::LowState_>>("rt/lowstate");
  low_state_subscriber->InitChannel(
      std::bind(&MotionTelemetry::callback, &telemetry,
                std::placeholders::_1), 1);
  SportTelemetry sport_telemetry;
  auto sport_state_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<
      unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
  sport_state_subscriber->InitChannel(
      std::bind(&SportTelemetry::callback, &sport_telemetry,
                std::placeholders::_1), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (call_stand_up)
  {
    const int32_t stand_result = sport_client.StandUp();
    std::cout << "StandUp result=" << stand_result << std::endl;
    if (stand_result != 0) return 5;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  else
  {
    std::cout << "StandUp skipped: testing the already-standing controller."
              << std::endl;
  }

  telemetry.reset();
  int failed_moves = 0;
  int32_t last_move_result = 0;
  auto next_tick = std::chrono::steady_clock::now();
  for (int i = 0; i < kCommandCount; ++i)
  {
    last_move_result = sport_client.Move(kTestVx, 0.0f, 0.0f);
    if (last_move_result != 0) ++failed_moves;
    next_tick += std::chrono::milliseconds(kPeriodMs);
    std::this_thread::sleep_until(next_tick);
  }
  sport_client.Move(0.0f, 0.0f, 0.0f);
  sport_client.StopMove();
  telemetry.stop();

  std::cout << "Move test complete: commands=" << kCommandCount
            << " period_ms=" << kPeriodMs
            << " vx=" << kTestVx
            << " failed_moves=" << failed_moves
            << " last_result=" << last_move_result << std::endl;
  std::cout << "Physical response: lowstate_samples=" << telemetry.samples()
            << " max_joint_dq_rad_s=" << telemetry.maxJointDq()
            << " min_foot_force=" << telemetry.minFootForce()
            << " saw_foot_unload="
            << (telemetry.sawFootUnload() ? "true" : "false") << std::endl;
  std::cout << "Remote during Move: valid_frame="
            << (telemetry.remoteValid() ? "true" : "false")
            << " button_activity="
            << (telemetry.remoteButtonActivity() ? "true" : "false")
            << " max_abs_axis=" << telemetry.maxRemoteAxis() << std::endl;
  std::cout << "Sport state during Move: samples=" << sport_telemetry.samples()
            << " error_code=" << sport_telemetry.errorCode()
            << " mode=" << sport_telemetry.mode()
            << " gait_type=" << sport_telemetry.gaitType()
            << " foot_raise_height=" << sport_telemetry.footRaiseHeight()
            << " max_reported_xy_speed="
            << sport_telemetry.maxReportedSpeed() << std::endl;

  if (failed_moves != 0) return 6;
  if (telemetry.maxJointDq() < 0.5 || !telemetry.sawFootUnload())
  {
    std::cerr << "RESULT: SDK accepted Move, but the legs did not enter a "
              << "walking cycle." << std::endl;
    return 7;
  }
  std::cout << "RESULT: official high-rate Move produced a walking response."
            << std::endl;
  return 0;
}
