#include <gtest/gtest.h>
#include "../src/sensor_continuity.h"
using go2_exploration::SensorContinuity;
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
TEST(SensorContinuity, ContinuousInputsPass) {
  SensorContinuity g;
  for (int i=0;i<100;++i) {
    const double stamp=100+i*.005;
    EXPECT_TRUE(g.accept(false,stamp,stamp+.001,stamp));
    if (i%20==0) EXPECT_TRUE(g.accept(true,stamp,stamp+.11,stamp,20064,.1003));
    EXPECT_TRUE(g.watchdog(stamp));
  }
}
TEST(SensorContinuity, PartialStartupSkippedButOversizedScanLatches) {
  SensorContinuity g;
  EXPECT_FALSE(g.accept(true,100,100.01,100,96,.00047));EXPECT_TRUE(g.fault.empty());
  EXPECT_TRUE(g.accept(true,100.1,100.21,100.2,20064,.1003));
  EXPECT_FALSE(g.accept(true,100.2,100.3,100.3,5856,2.440467917));
  EXPECT_EQ("lidar_invalid_scan_span_or_size",g.fault);
  EXPECT_FALSE(g.accept(true,100.3,100.41,100.4,20064,.1));
}
TEST(SensorContinuity, MissingImuStopsBothStreamsWithoutAutomaticRecovery) {
  SensorContinuity g;
  ASSERT_TRUE(g.accept(false,100,100,100));
  EXPECT_FALSE(g.watchdog(100.251));EXPECT_EQ("imu_stream_timeout",g.fault);
  EXPECT_FALSE(g.accept(false,100.3,100.3,100.3));
  EXPECT_FALSE(g.accept(true,100.3,100.4,100.4,20000,.1));
}
TEST(SensorContinuity, TimeReversalAndLongGapRejectedBeforeEstimator) {
  SensorContinuity g;
  ASSERT_TRUE(g.accept(true,100,100.1,100.1,20000,.1));
  EXPECT_FALSE(g.accept(true,99.999,100.1,100.2,20000,.1));
  SensorContinuity h;
  ASSERT_TRUE(h.accept(false,100,100,100));
  EXPECT_FALSE(h.accept(false,102.4199,102.42,102.42));
}
