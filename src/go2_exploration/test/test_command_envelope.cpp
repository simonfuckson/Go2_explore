#include <gtest/gtest.h>
#include "../src/command_envelope.h"
#include "../src/forward_connection.h"
using namespace go2_exploration;
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
TEST(CommandEnvelope, SideUnknownRejectsActualSteppingTurn) {
  // Observed startup corridor: the body fits, but cells at |y| >= .225
  // remain unknown. Raw .04 rad/s fits; GO2's .50 rad/s sweep does not.
  const auto corridor=[](double,double y){return std::abs(y)<.225;};
  const PlanarCommand raw{0,-.04};const auto target=shapedTarget(raw,ShaperLimits{});
  EXPECT_TRUE(sweepClear(0,0,0,raw,SweepGeometry{},.05,corridor));
  EXPECT_DOUBLE_EQ(target.w,-.5);
  EXPECT_FALSE(sweepClear(0,0,0,target,SweepGeometry{},.05,corridor));
}
TEST(CommandEnvelope, ForwardFloorIncludesLongerStoppingDistance) {
  const auto wall=[](double x,double){return x<.60;};
  const PlanarCommand raw{.1,0};const auto target=shapedTarget(raw,ShaperLimits{});
  EXPECT_TRUE(sweepClear(0,0,0,raw,SweepGeometry{},.05,wall));
  EXPECT_DOUBLE_EQ(target.v,.3);
  EXPECT_FALSE(sweepClear(0,0,0,target,SweepGeometry{},.05,wall));
}
TEST(CommandEnvelope, FreeSpaceAndZeroRemainUsable) {
  const auto free=[](double,double){return true;};
  for(const auto raw:{PlanarCommand{0,0},PlanarCommand{0,.1},PlanarCommand{.1,-.1}})
    EXPECT_TRUE(sweepClear(0,0,.7,shapedTarget(raw,ShaperLimits{}),SweepGeometry{},.05,free));
  const auto zero=shapedTarget({0,0},ShaperLimits{});
  EXPECT_DOUBLE_EQ(zero.v,0);EXPECT_DOUBLE_EQ(zero.w,0);
}
TEST(CommandEnvelope, SDKFloorAndConfiguredLimitsAreRespected) {
  ShaperLimits p;p.walk_floor=.2;p.turn_floor=.4;p.max_v=.25;p.max_w=.45;
  auto out=shapedTarget({.02,.02},p);
  EXPECT_DOUBLE_EQ(out.v,.2);EXPECT_DOUBLE_EQ(out.w,.04);
  out=shapedTarget({0,-.02},p);EXPECT_DOUBLE_EQ(out.w,-.4);
  out=shapedTarget({.3,.5},p);EXPECT_DOUBLE_EQ(out.v,.25);EXPECT_DOUBLE_EQ(out.w,.45);
}
TEST(ForwardConnection, UsesObservedForwardCorridorWhenTurnIsUnknown) {
  const auto free=[](double x,double y){return (x>=-.40 && std::abs(y)<.225) || (x>.35 && std::abs(y)<1.);};
  EXPECT_FALSE(sweepClear(0,0,0,{0,.5},SweepGeometry{},.05,free));
  PlanarCommand cmd;
  ASSERT_TRUE(forwardConnection({{0,0},{.1,0},{.3,0},{1.,.3}},ShaperLimits{},
      [&](PlanarCommand v){return sweepClear(0,0,0,v,SweepGeometry{},.05,free);},cmd));
  EXPECT_DOUBLE_EQ(cmd.v,.30);EXPECT_LE(std::abs(cmd.w),.3);
}
TEST(ForwardConnection, CannotEscapeThroughObstacleOrUnknownOrAwayFromRoute) {
  PlanarCommand cmd;const std::vector<RoutePoint> route{{0,0},{1,0}};
  const auto barrier=[](double x,double){return x<.55;};
  EXPECT_FALSE(forwardConnection(route,ShaperLimits{},[&](PlanarCommand v){return sweepClear(0,0,0,v,SweepGeometry{},.05,barrier);},cmd));
  EXPECT_FALSE(forwardConnection({{0,0},{-1,0}},ShaperLimits{},[](PlanarCommand){return true;},cmd));
  EXPECT_FALSE(forwardConnection({{0,0},{.2,0}},ShaperLimits{},[](PlanarCommand){return true;},cmd));
  EXPECT_FALSE(forwardConnection(route,ShaperLimits{},[](PlanarCommand){return false;},cmd));
}
TEST(CommandEnvelope, StartupIntermediateArcCannotBypassEndpointChecks) {
  const auto corridor=[](double x,double y){return std::abs(y)<.2 || x>.20;};
  const auto clear=[&](PlanarCommand v){return sweepClear(0,0,0,v,SweepGeometry{},.05,corridor);};
  ASSERT_TRUE(clear({0,0}));ASSERT_TRUE(clear({.3,-.2}));
  EXPECT_FALSE(transitionClear({0,0},{.3,-.2},ShaperLimits{},clear));
  EXPECT_TRUE(transitionClear({0,0},{.3,0},ShaperLimits{},clear));
  EXPECT_TRUE(transitionClear({.3,0},{.3,-.2},ShaperLimits{},clear));
}
