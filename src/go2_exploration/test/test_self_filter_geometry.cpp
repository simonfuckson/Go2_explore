#include <gtest/gtest.h>
#include "../src/self_filter_geometry.h"
using namespace go2_exploration;
SelfBox box() {SelfBox b;b.name="measured_camera";b.center={.45,0,.05};b.size={.04,.1,.04};b.padding=.005;return b;}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
TEST(SelfFilter, OnlyMeasuredBodyAndOccludedRaysAreRejected) {
  SelfFilterGeometry g;g.origin={.187,0,.16};g.boxes={box()};g.validate();
  EXPECT_TRUE(g.reject({.45,0,.05}));
  EXPECT_TRUE(g.reject(g.origin+2*(Eigen::Vector3d(.45,0,.05)-g.origin)));
  EXPECT_FALSE(g.reject({.42,0,.07})); // real object before the mount
  EXPECT_FALSE(g.reject({.45,.061,.05})); // real object just beside it
  EXPECT_FALSE(g.reject({.45,0,.076})); // real object just above it
  EXPECT_FALSE(g.reject({.45,0,-.20})); // floor below the camera
}
TEST(SelfFilter, RotatedThinBracketDoesNotMaskAnAxisAlignedCube) {
  SelfBox b=box();b.size={.20,.02,.02};b.padding=0;b.rotation=rpyRotation(0,0,M_PI/4);b.validate();
  EXPECT_TRUE(b.contains(b.center+b.rotation*Eigen::Vector3d(.08,0,0)));
  EXPECT_FALSE(b.contains(b.center+Eigen::Vector3d(.07,0,0)));
}
TEST(SelfFilter, OriginalLidarPitchIsUsedWithoutTouchingCalibration) {
  SelfFilterGeometry g;g.origin={.187,0,.16};g.lidar_rotation=rpyRotation(-.1*M_PI/180,39*M_PI/180,0);
  auto p=g.toBase({1,0,0});EXPECT_NEAR(p.x(),.187+std::cos(39*M_PI/180),1e-10);
  EXPECT_NEAR(p.z(),.16-std::sin(39*M_PI/180),1e-10);
}
TEST(SelfFilter, InvalidAndOriginCoveringGeometryIsRejected) {
  auto b=box();b.padding=.04;EXPECT_THROW(b.validate(),std::invalid_argument);
  b=box();b.size.x()=0;EXPECT_THROW(b.validate(),std::invalid_argument);
  SelfFilterGeometry g;g.origin=b.center;g.boxes={box()};EXPECT_THROW(g.validate(),std::invalid_argument);
  b=box();b.center.x()=NAN;EXPECT_THROW(b.validate(),std::invalid_argument);
}
TEST(SelfFilter, NoGeometryPreservesAllSceneReturns) {
  SelfFilterGeometry g;g.validate();
  for (double x=.1;x<2;x+=.1) EXPECT_FALSE(g.reject({x,.1,.2}));
}
