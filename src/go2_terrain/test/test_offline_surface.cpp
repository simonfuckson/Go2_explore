#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include "go2_terrain/offline_surface.hpp"
#include "go2_terrain/terrain_algorithms.hpp"
#include "go2_terrain/export_transaction.hpp"

using namespace go2_terrain;
namespace {
GridGeometry geometry() {
  GridGeometry g; g.width=100; g.height=60; g.resolution=.05;
  g.origin_x=-1; g.origin_y=-1.5; return g;
}
SurfaceParameters parameters() { SurfaceParameters p; p.seed_ground_z=-.35; return p; }
pcl::PointCloud<pcl::PointXYZ> plane(double degrees=0, bool ceiling=false, bool stripes=false) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  auto g=geometry();
  for (int y=2;y<58;++y) for (int x=2;x<98;++x) {
    if (stripes && x%20>=17) continue;
    const float wx=g.origin_x+(x+.5)*g.resolution;
    const float wy=g.origin_y+(y+.5)*g.resolution;
    const float z=-.35+std::tan(degrees*M_PI/180)*wx;
    cloud.push_back(pcl::PointXYZ(wx,wy,z));
    if (ceiling) cloud.push_back(pcl::PointXYZ(wx,wy,2.55));
  }
  return cloud;
}
std::size_t at(double x, double y) {
  auto g=geometry();return std::floor((y-g.origin_y)/g.resolution)*g.width+
      std::floor((x-g.origin_x)/g.resolution);
}
}
TEST(OfflineSurface, RampAnglesRemainGroundAndOnlySteepRampIsLethal) {
  for (double angle: {0.,10.,20.,30.,35.}) {
    auto r=reconstructSurface(plane(angle),geometry(),parameters());
    const auto i=at(1,0);
    ASSERT_TRUE(isKnown(r.terrain.elevation[i])) << angle;
    EXPECT_NEAR(r.terrain.slope_deg[i],angle,.05);
    EXPECT_LT(r.terrain.step[i],.001);
    EXPECT_EQ(0U,r.obstacle_count[i]);
    CostParameters costs;
    auto cost=buildSlopeCostLayer(r.terrain.slope_deg,geometry(),costs);
    if (angle>30) EXPECT_EQ(254,cost[i]);
    else EXPECT_LT(cost[i],254);
  }
}
TEST(OfflineSurface, SparseStripesDoNotCutContinuousRamp) {
  auto r=reconstructSurface(plane(20,false,true),geometry(),parameters());
  EXPECT_TRUE(isKnown(r.terrain.elevation[at(.90,0)]));
  EXPECT_NEAR(20,r.terrain.slope_deg[at(.90,0)],.1);
}
TEST(OfflineSurface, RampCrestAndFootRemainConnected) {
  auto cloud=plane();
  for(auto& point:cloud) point.z=-.35+std::tan(20*M_PI/180)*
      std::max(0.,std::min(point.x-.5,2.5-point.x));
  auto r=reconstructSurface(cloud,geometry(),parameters());
  for (double x=.1;x<3;x+=.05) {
    EXPECT_TRUE(isKnown(r.terrain.elevation[at(x,0)])) << x;
    EXPECT_EQ(0U,r.obstacle_count[at(x,0)]) << x;
  }
}
TEST(OfflineSurface, CeilingAboveMeasuredFloorDoesNotMarkOrBecomeGround) {
  auto r=reconstructSurface(plane(0,true),geometry(),parameters());
  for (const auto& p:r.ground) EXPECT_NEAR(-.35,p.z,.002);
  EXPECT_TRUE(r.obstacles.empty());
}
TEST(OfflineSurface, VerticalWallAndThinPostSurvive) {
  auto cloud=plane();
  for (int j=-20;j<=20;++j) for (int k=0;k<20;++k)
    cloud.push_back(pcl::PointXYZ(2.025,j*.05+.025,-.20+k*.08));
  for (int k=0;k<15;++k) cloud.push_back(pcl::PointXYZ(.625,.225,-.20+k*.08));
  auto r=reconstructSurface(cloud,geometry(),parameters());
  EXPECT_GE(r.obstacle_count[at(2.025,.025)],8U);
  EXPECT_GE(r.obstacle_count[at(.625,.225)],8U);
}
TEST(OfflineSurface, DisconnectedCeilingCannotBecomeFloor) {
  auto cloud=plane();
  for (auto& p:cloud) if (p.x>2) p.z=2.55;
  auto r=reconstructSurface(cloud,geometry(),parameters());
  EXPECT_FALSE(isKnown(r.terrain.elevation[at(3,0)]));
}
TEST(OfflineSurface, SeparateFloorCeilingReturnsAreNotVerticalSupport) {
  EXPECT_FALSE(hasVerticalSupport({-.35,-.34,2.54,2.55}));
  EXPECT_FALSE(hasVerticalSupport({.1,.1,.1,.9}));
  EXPECT_TRUE(hasVerticalSupport({.01,.12,.23,.34,.45}));
  EXPECT_FALSE(hasVerticalSupport({NAN,INFINITY}));
}
TEST(OfflineSurface, LowStepChainRemainsAnObstacle) {
  for (double height: {.05,.08,.10}) {
    auto cloud=plane();
    for (auto& p:cloud) if (p.x>1)
      p.z+=height*std::floor((p.x-1)/.25+1);
    auto r=reconstructSurface(cloud,geometry(),parameters());
    unsigned count=0;
    for (int y=10;y<50;++y) for (int x=40;x<75;++x)
      count+=r.obstacle_count[y*geometry().width+x];
    EXPECT_GT(count,0U) << height;
  }
}
TEST(OfflineSurface, EmptyAndOversizeInputsFail) {
  EXPECT_THROW(reconstructSurface({},geometry(),parameters()),std::runtime_error);
  auto g=geometry();g.width=100000;g.height=100000;
  EXPECT_THROW(reconstructSurface(plane(),g,parameters()),std::runtime_error);
}
TEST(OfflineSurface, LargeDisconnectedRoofIsNotADistributedFloorAnchor) {
  auto g=geometry();g.width=220;g.height=160;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int y=2;y<158;++y) for (int x=2;x<218;++x) {
    const float wx=g.origin_x+(x+.5)*g.resolution;
    const float wy=g.origin_y+(y+.5)*g.resolution;
    cloud.push_back(pcl::PointXYZ(wx,wy,wx<1.5 ? -.35 : 2.55));
  }
  auto r=reconstructSurface(cloud,g,parameters());
  EXPECT_FALSE(isKnown(r.terrain.elevation[80*g.width+150]));
}
TEST(ExportTransaction, BadAssetPreservesAllOldAssets) {
  namespace fs=boost::filesystem;
  auto root=fs::temp_directory_path()/fs::unique_path("go2-export-test-%%%%-%%%%");
  fs::create_directories(root/"stage"); fs::create_directory(root/"map");
  std::ofstream((root/"stage"/"a").string())<<"new";
  std::ofstream((root/"map"/"a").string())<<"old";
  EXPECT_THROW(commitExportFiles(root/"stage",root/"map",{"a","missing"}),std::runtime_error);
  std::string value;std::ifstream((root/"map"/"a").string())>>value;
  EXPECT_EQ("old",value);fs::remove_all(root);
}
TEST(ExportTransaction, RenameFailureRollsBackAlreadyReplacedFile) {
  namespace fs=boost::filesystem;
  auto root=fs::temp_directory_path()/fs::unique_path("go2-export-test-%%%%-%%%%");
  fs::create_directories(root/"stage"); fs::create_directory(root/"map");
  std::ofstream((root/"stage"/"a").string())<<"new";
  std::ofstream((root/"map"/"a").string())<<"old";
  std::ofstream((root/"stage"/"b").string())<<"new-b";
  std::ofstream((root/"map"/"b").string())<<"old-b";
  EXPECT_ANY_THROW(commitExportFiles(root/"stage",root/"map",{"a","b"},
      [](std::size_t i){if(i==1) throw std::runtime_error("injected I/O failure");}));
  std::string value;std::ifstream((root/"map"/"a").string())>>value;
  EXPECT_EQ("old",value);fs::remove_all(root);
}
