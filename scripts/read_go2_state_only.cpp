// Passive DDS subscriber only. No publisher, SportClient or motion API.
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>
int main() {
  unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
  std::mutex mutex;unsigned count=0;unitree_go::msg::dds_::SportModeState_ latest;
  unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sub("rt/sportmodestate");
  sub.InitChannel([&](const void* data){std::lock_guard<std::mutex> lock(mutex);latest=*static_cast<const unitree_go::msg::dds_::SportModeState_*>(data);++count;},1);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::lock_guard<std::mutex> lock(mutex);
  std::cout<<"{\"samples\":"<<count<<",\"mode\":"<<unsigned(latest.mode())<<",\"gait_type\":"<<unsigned(latest.gait_type())<<",\"body_height\":"<<latest.body_height()<<",\"foot_position_body\":[";
  for(unsigned i=0;i<12;++i)std::cout<<(i?",":"")<<latest.foot_position_body()[i];
  std::cout<<"],\"foot_force\":[";
  for(unsigned i=0;i<4;++i)std::cout<<(i?",":"")<<latest.foot_force()[i];
  std::cout<<"]}"<<std::endl;
}
