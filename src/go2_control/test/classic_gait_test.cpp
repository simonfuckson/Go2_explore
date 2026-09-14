#include <gtest/gtest.h>
#include <go2_control/classic_gait.hpp>
#include <go2_control/manual_control.hpp>
#include <string>
#include <vector>
namespace {
struct FakeSport {
  int move_code=0, classic_code=0;
  std::vector<int> replies;
  size_t reply_index=0;
  std::vector<std::string> calls;
  int Move(float x,float y,float yaw) {
    EXPECT_FLOAT_EQ(0,x); EXPECT_FLOAT_EQ(0,y); EXPECT_FLOAT_EQ(0,yaw);
    calls.push_back("zero"); return move_code;
  }
  int ClassicWalk(bool on) {
    calls.push_back(on ? "classic_on" : "classic_off");
    return reply_index < replies.size() ? replies[reply_index++] : classic_code;
  }
};
}
TEST(ClassicGait, EnableExplicitlyRequestsClassicAfterZero) {
  FakeSport sdk;
  EXPECT_TRUE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ((std::vector<std::string>{"zero","classic_on"}),sdk.calls);
}
TEST(ClassicGait, FailedZeroCannotSelectGait) {
  FakeSport sdk; sdk.move_code=3102;
  EXPECT_FALSE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ((std::vector<std::string>{"zero"}),sdk.calls);
}
TEST(ClassicGait, ClassicRejectionCannotArmOrFallback) {
  for(int code:{3102,3103,3104,3105,3106,3107,4205,7004}) {
    FakeSport sdk; sdk.classic_code=code;
    const auto r=go2_control::requestClassicWalk(sdk);
    EXPECT_FALSE(r.accepted()); EXPECT_EQ(code,r.classic_result);
    EXPECT_EQ(2U,sdk.calls.size());
  }
}
TEST(ClassicGait, GenericRejectionRequiresAcknowledgedOffAndOn) {
  FakeSport sdk; sdk.replies={-1,0,0};
  const auto r=go2_control::requestClassicWalk(sdk);
  EXPECT_TRUE(r.accepted()); EXPECT_TRUE(r.reset_attempted);
  EXPECT_EQ(-1,r.first_classic_result);
  EXPECT_EQ((std::vector<std::string>{"zero","classic_on","classic_off","classic_on"}),sdk.calls);
}
TEST(ClassicGait, ResetFailureCannotArm) {
  FakeSport sdk; sdk.replies={-1,-1};
  EXPECT_FALSE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ(3U,sdk.calls.size());
}
TEST(ClassicGait, RetryIsBoundedAndNeverAcceptsMinusOne) {
  FakeSport sdk; sdk.replies={-1,0,-1};
  EXPECT_FALSE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ(4U,sdk.calls.size());
}
TEST(ClassicGait, RemoteInterruptionDuringResetPreventsOn) {
  FakeSport sdk; sdk.replies={-1,0,0}; bool permit=true;
  const auto r=go2_control::requestClassicWalk(sdk,[&]{return permit;},[&]{permit=false;});
  EXPECT_FALSE(r.accepted()); EXPECT_TRUE(r.cancelled);
  EXPECT_EQ(3U,sdk.calls.size());
}
TEST(ClassicGait, JoystickRoutingIsNotGaitSelection) {
  using J=go2_control::JoystickRequest;
  EXPECT_FALSE(go2_control::changesSportMode(1027));
  EXPECT_EQ(J::kReleased,go2_control::joystickRequest(1027,true,false));
  EXPECT_EQ(J::kTakeover,go2_control::joystickRequest(1027,true,true));
  EXPECT_EQ(J::kTakeover,go2_control::joystickRequest(1027,false,false));
  EXPECT_EQ(J::kNotJoystick,go2_control::joystickRequest(2049,true,true));
}
TEST(ManualControl, BootNeutralNeverArms) {
  go2_control::ManualControlState s;
  for(int i=0;i<=15;++i) s.observe(0,0,0,0,0,i*.1,false);
  EXPECT_TRUE(s.neutralAndFresh(1.5)); EXPECT_FALSE(s.canResume(1.5));
}
TEST(ManualControl, DecodesActualRobotLowStatePacket) {
  std::array<uint8_t,40> bytes{}; bytes[0]=0x55; bytes[1]=0x51;
  // Actual centered packet uses negative zero for several axes.
  bytes[7]=bytes[11]=bytes[15]=0x80;
  go2_control::RemoteSample s;
  ASSERT_TRUE(go2_control::decodeLowStateRemote(bytes,s));
  EXPECT_FLOAT_EQ(0,s.lx); EXPECT_FLOAT_EQ(0,s.ly); EXPECT_EQ(0,s.keys);
  bytes[2]=0x01; bytes[3]=0x08;
  const float lx=.25f, rx=-.5f, ry=.75f, ly=1.f;
  std::memcpy(bytes.data()+4,&lx,4); std::memcpy(bytes.data()+8,&rx,4);
  std::memcpy(bytes.data()+12,&ry,4); std::memcpy(bytes.data()+20,&ly,4);
  ASSERT_TRUE(go2_control::decodeLowStateRemote(bytes,s));
  EXPECT_EQ(0x0801,s.keys); EXPECT_FLOAT_EQ(lx,s.lx); EXPECT_FLOAT_EQ(rx,s.rx);
  EXPECT_FLOAT_EQ(ry,s.ry); EXPECT_FLOAT_EQ(ly,s.ly);
}
TEST(ManualControl, InvalidRemoteHeaderCannotBecomeNeutral) {
  std::array<uint8_t,40> bytes{};
  go2_control::RemoteSample s;
  EXPECT_FALSE(go2_control::decodeLowStateRemote(bytes,s));
}
TEST(ManualControl, RetainsPermissionUntilContinuousNeutral) {
  go2_control::ManualControlState s;
  s.observe(.5,0,0,0,0,0,true);
  EXPECT_TRUE(s.resumable); EXPECT_FALSE(s.canResume(0));
  for(int i=1;i<=15;++i) s.observe(0,0,0,0,0,i*.1,false);
  EXPECT_TRUE(s.canResume(1.5)); EXPECT_FALSE(s.canResume(2.1));
  s.observe(0,0,0,0,0,2.2,false); EXPECT_FALSE(s.canResume(2.2));
}
TEST(ManualControl, KeysAndExplicitDisableBlockResume) {
  for(bool key:{false,true}) {
    go2_control::ManualControlState s;
    s.observe(.5,0,0,0,0,0,true);
    if(key) s.observe(0,0,0,0,1,.1,false); else s.cancelResume();
    for(int i=2;i<=20;++i) s.observe(0,0,0,0,0,i*.1,false);
    EXPECT_FALSE(s.canResume(2.0));
  }
}
TEST(ManualControl, StickMovementWhileExplicitlyDisabledCannotArm) {
  go2_control::ManualControlState s;
  s.observe(0,.5,0,0,0,0,false);
  for(int i=1;i<=20;++i) s.observe(0,0,0,0,0,i*.1,false);
  EXPECT_FALSE(s.canResume(2.0));
}
TEST(ClassicGait, EveryNewEnableReappliesClassic) {
  FakeSport sdk;
  EXPECT_TRUE(go2_control::requestClassicWalk(sdk).accepted());
  sdk.calls.clear();
  EXPECT_TRUE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ((std::vector<std::string>{"zero","classic_on"}),sdk.calls);
}
TEST(ClassicGait, CommandsThatChangeGaitOrPostureInvalidatePolicy) {
  for(int api:{1001,1003,1004,1005,1028,1061,1062,1063,2045,2049})
    EXPECT_TRUE(go2_control::changesSportMode(api));
  for(int api:{1008,1015,1034,2055})
    EXPECT_FALSE(go2_control::changesSportMode(api));
}
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS();
}
