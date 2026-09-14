#include <gtest/gtest.h>
#include <explore/boundary_goal.h>
#include <explore/frontier_search.h>
#include <explore/known_space.h>
#include <costmap_2d/cost_values.h>
#include <explore/view_gain.h>
#include <explore/anchored_path.h>
#include <explore/arrival_view.h>

TEST(BoundaryGoal, RingCenterCannotBeReturnedAsGoal) {
  geometry_msgs::Point robot;
  std::vector<geometry_msgs::Point> points;
  for (int i=0;i<24;++i) {
    geometry_msgs::Point p;
    p.x=.225*std::cos(i*2*M_PI/24);
    p.y=.225*std::sin(i*2*M_PI/24);
    points.push_back(p);
  }
  const auto result=explore::boundaryCandidates(points,robot,robot,0,.20);
  ASSERT_FALSE(result.empty());
  EXPECT_NEAR(result.front().x,.225,1e-9);
  EXPECT_NEAR(result.front().y,0,1e-9);
  for (const auto& p:result) EXPECT_GE(std::hypot(p.x,p.y),.20);
}

TEST(BoundaryGoal, TinyFrontierWaitsInsteadOfReportingArrival) {
  geometry_msgs::Point robot,p;
  p.x=.10;
  EXPECT_TRUE(explore::boundaryCandidates({p},robot,robot,0,.20).empty());
}

TEST(FrontierGeometry, SeedIsIncludedAtTranslatedOrigin) {
  costmap_2d::Costmap2D map(21,21,.05,10,20,costmap_2d::NO_INFORMATION);
  for (unsigned y=8;y<=12;++y)
    for (unsigned x=8;x<=12;++x) map.setCost(x,y,costmap_2d::FREE_SPACE);
  geometry_msgs::Point robot;
  map.mapToWorld(10,10,robot.x,robot.y);
  frontier_exploration::FrontierSearch search(&map,.001,1,.5);
  const auto found=search.searchFrom(robot);
  ASSERT_EQ(found.size(),1u);
  EXPECT_EQ(found.front().size,found.front().points.size());
  EXPECT_NEAR(found.front().centroid.x,robot.x,1e-9);
  EXPECT_NEAR(found.front().centroid.y,robot.y,1e-9);
}

TEST(KnownApproach, ChassisMustFitInsideObservedArea) {
  costmap_2d::Costmap2D map(60,60,.05,-1.5,-1.5,costmap_2d::NO_INFORMATION);
  for (unsigned y=10;y<50;++y) for (unsigned x=10;x<50;++x)
    map.setCost(x,y,costmap_2d::FREE_SPACE);
  EXPECT_TRUE(explore::knownFootprint(map,0,0,0,.665,.38,.185));
  EXPECT_FALSE(explore::knownFootprint(map,.9,0,0,.665,.38,.185));
  geometry_msgs::Point p;
  map.mapToWorld(50,30,p.x,p.y);
  const auto goals=explore::knownApproaches(map,{p});
  ASSERT_FALSE(goals.empty());
  for (const auto& g:goals) {
    unsigned x,y;
    ASSERT_TRUE(map.worldToMap(g.x,g.y,x,y));
    EXPECT_LT(map.getCost(x,y),costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
}

TEST(KnownApproach, InteriorObstacleAndUnknownAreRejectedButSoftCostsPreserved) {
  costmap_2d::Costmap2D map(60,60,.05,-1.5,-1.5,100);
  EXPECT_TRUE(explore::knownFootprint(map,0,0,0,.38,.38,.185));
  map.setCost(31,31,costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(explore::knownFootprint(map,0,0,0,.38,.38,.185));
  map.setCost(31,31,costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(explore::knownFootprint(map,0,0,0,.38,.38,.185));
}

TEST(ViewGain, RearGroundIsBlindAndForwardGroundDependsOnMounting) {
  EXPECT_FALSE(explore::groundInView(-2,0,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.));
  EXPECT_TRUE(explore::groundInView(2,0,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.));
  EXPECT_FALSE(explore::groundInView(.2,0,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.));
  EXPECT_FALSE(explore::groundInView(2,0,3,.51,0,.187,0,0,-.1*M_PI/180.));
}

TEST(ViewGain, FacingUnknownHasGainButKnownWallOccludesIt) {
  costmap_2d::Costmap2D map(140,140,.05,-3.5,-3.5,costmap_2d::FREE_SPACE);
  for (unsigned y=0;y<140;++y) for (unsigned x=100;x<140;++x)
    map.setCost(x,y,costmap_2d::NO_INFORMATION);
  EXPECT_GT(explore::expectedGroundGain(map,0,0,0,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.),0.1);
  EXPECT_NEAR(explore::expectedGroundGain(map,0,0,M_PI,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.),0,1e-6);
  for (unsigned y=0;y<140;++y) map.setCost(90,y,costmap_2d::LETHAL_OBSTACLE);
  EXPECT_NEAR(explore::expectedGroundGain(map,0,0,0,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.),0,1e-6);
}

TEST(CoverageFrontiers, UnobservedStartDoesNotDisconnectCollisionFreeRoom) {
  costmap_2d::Costmap2D collision(40,40,.05,-1,-1,costmap_2d::FREE_SPACE);
  costmap_2d::Costmap2D interest(40,40,.05,-1,-1,costmap_2d::FREE_SPACE);
  for (unsigned y=8;y<30;++y) for (unsigned x=5;x<22;++x)
    interest.setCost(x,y,costmap_2d::NO_INFORMATION);
  geometry_msgs::Point robot;
  frontier_exploration::FrontierSearch search(&interest,.001,1,.5,&collision);
  const auto found=search.searchFrom(robot);
  ASSERT_EQ(found.size(),1u);
  EXPECT_EQ(found.front().size,22u*17u);
  EXPECT_TRUE(explore::knownFootprint(collision,0,0,0,.38,.38,.185));
  EXPECT_FALSE(explore::knownFootprint(interest,0,0,0,.38,.38,.185));
  // A collision wall still prevents entering an unrelated room.
  for (unsigned y=0;y<40;++y) collision.setCost(23,y,costmap_2d::LETHAL_OBSTACLE);
  for (unsigned y=0;y<40;++y) for (unsigned x=25;x<39;++x)
    interest.setCost(x,y,costmap_2d::NO_INFORMATION);
  EXPECT_EQ(search.searchFrom(robot).size(),1u);
}

TEST(ViewGain, CoverageUnknownHasGainEvenWhenCollisionMapWasCleared) {
  costmap_2d::Costmap2D collision(140,140,.05,-3.5,-3.5,costmap_2d::FREE_SPACE);
  costmap_2d::Costmap2D coverage(140,140,.05,-3.5,-3.5,costmap_2d::FREE_SPACE);
  for (unsigned y=0;y<140;++y) for (unsigned x=0;x<40;++x)
    coverage.setCost(x,y,costmap_2d::NO_INFORMATION);
  EXPECT_NEAR(explore::expectedGroundGain(collision,0,0,0,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.,&coverage),0.,1e-6);
  EXPECT_GT(explore::expectedGroundGain(collision,0,0,M_PI,3,.51,39*M_PI/180.,.187,0,0,-.1*M_PI/180.,&coverage),.1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}

TEST(GroundVisibility, RespectsFullMountRotation) {
  EXPECT_TRUE(explore::groundInView(.187,2,3,.51,39*M_PI/180.,.187,0,M_PI/2,0));
  EXPECT_FALSE(explore::groundInView(2,0,3,.51,39*M_PI/180.,.187,0,M_PI,0));
  EXPECT_FALSE(explore::groundInView(2,0,3,.51,39*M_PI/180.,.187,0,0,M_PI));
}

TEST(AnchoredPath, GridCenterBehindMeasuredStartDoesNotInventReverseMotion) {
  costmap_2d::Costmap2D map(80,80,.05,-2,-2,costmap_2d::FREE_SPACE);
  for (unsigned y=0;y<80;++y) for (unsigned x=0;x<32;++x)
    map.setCost(x,y,costmap_2d::NO_INFORMATION);
  geometry_msgs::Pose robot;
  robot.position.x=-.004;robot.position.y=-.013;
  robot.orientation=tf::createQuaternionMsgFromYaw(.007);
  std::vector<geometry_msgs::PoseStamped> raw(12);
  for (size_t i=0;i<raw.size();++i) {
    raw[i].pose.position.x=-.025+i*.05;raw[i].pose.position.y=-.025;
  }
  // Cell-center occupancy does not turn a five-millimetre raster edge into
  // an occupied body cell; anchoring must still preserve forward progress.
  EXPECT_TRUE(explore::knownFootprint(map,-.025,-.025,0,.38,.38,.185));
  const auto path=explore::anchoredPath(raw,robot,.05);
  ASSERT_GT(path.size(),raw.size());
  EXPECT_DOUBLE_EQ(path.front().x,robot.position.x);
  EXPECT_DOUBLE_EQ(path.front().y,robot.position.y);
  EXPECT_NEAR(path.front().yaw,.007,1e-9);
  for (const auto& p:path) {
    EXPECT_GE(p.x,robot.position.x);
    EXPECT_TRUE(explore::knownFootprint(map,p.x,p.y,p.yaw,.38,.38,.185));
  }
  // Fixing the quantized prefix must still reject a measured obstacle ahead.
  unsigned mx,my;ASSERT_TRUE(map.worldToMap(.60,.075,mx,my));
  map.setCost(mx,my,costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(std::any_of(path.begin(),path.end(),[&](const auto& p) {
    return !explore::knownFootprint(map,p.x,p.y,p.yaw,.38,.38,.185);
  }));
}

TEST(AnchoredPath, ChecksConnectionAndRotationAndPreservesLaterReturn) {
  geometry_msgs::Pose robot;robot.orientation.w=1;
  std::vector<geometry_msgs::PoseStamped> raw(5);
  raw[0].pose.position.x=.025;
  raw[1].pose.position.x=.075;
  raw[2].pose.position.x=.20;raw[2].pose.position.y=.10;
  raw[3].pose.position.x=0;raw[3].pose.position.y=0;
  raw[4].pose.position.x=.5;
  const auto path=explore::anchoredPath(raw,robot,.05);
  ASSERT_FALSE(path.empty());
  for (size_t i=1;i<path.size();++i) {
    EXPECT_LE(std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y),.0250001);
    EXPECT_LE(std::abs(path[i].yaw-path[i-1].yaw),.050001);
  }
  EXPECT_TRUE(std::any_of(path.begin()+1,path.end(),[](const auto& p) {return p.x==0 && p.y==0;}));
  EXPECT_DOUBLE_EQ(path.back().x,.5);
}
TEST(ArrivalView, InfeasibleHighestGainDoesNotHideReachableLowerGain) {
  double yaw=0,gain=0;
  ASSERT_TRUE(explore::feasibleArrivalView({1.57,0.},0.,
      [](double heading){return std::abs(heading)<.2;},
      [](double heading){return heading>1?5.:1.;},yaw,gain));
  EXPECT_DOUBLE_EQ(yaw,0);EXPECT_DOUBLE_EQ(gain,1);
}
TEST(ArrivalView, RotationInteriorAndMissingGainStillPreventSelection) {
  double yaw=0,gain=0;
  EXPECT_FALSE(explore::feasibleArrivalView({1.},0.,
      [](double heading){return heading<.3 || heading>.7;},[](double){return 1.;},yaw,gain));
  EXPECT_FALSE(explore::feasibleArrivalView({0.},0.,
      [](double){return true;},[](double){return 0.;},yaw,gain));
}
TEST(KnownApproach, GridCellsOutsidePaddedBodyDoNotTrapStartingPose) {
  const double x=-.0206955943,y=-.0146147781,yaw=-.0231577521;
  costmap_2d::Costmap2D map(80,80,.05,-2,-2,costmap_2d::FREE_SPACE);
  unsigned mx,my;ASSERT_TRUE(map.worldToMap(-.4002463701,.0091803593,mx,my));
  map.setCost(mx,my,costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(explore::knownFootprint(map,x,y,yaw,.38,.38,.185));
  ASSERT_TRUE(map.worldToMap(x,y,mx,my));map.setCost(mx,my,costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(explore::knownFootprint(map,x,y,yaw,.38,.38,.185));
  map.setCost(mx,my,costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(explore::knownFootprint(map,x,y,yaw,.38,.38,.185));
  EXPECT_FALSE(explore::knownFootprint(map,-1.9,0,0,.38,.38,.185));
}
