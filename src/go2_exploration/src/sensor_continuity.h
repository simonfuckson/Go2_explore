#pragma once
#include <cmath>
#include <string>

namespace go2_exploration {
// A broken input stream invalidates the current estimator session. Never stitch
// scans across a disconnect or silently resume an estimator with missing IMU.
class SensorContinuity {
 public:
  std::string fault;
  double lidar_stamp=0, imu_stamp=0, lidar_receipt=0, imu_receipt=0;
  bool accept(bool lidar, double stamp, double now_ros, double now_wall,
              unsigned points=0, double span=0) {
    if (!fault.empty()) return false;
    double& previous=lidar ? lidar_stamp : imu_stamp;
    double& receipt=lidar ? lidar_receipt : imu_receipt;
    const std::string stream=lidar ? "lidar" : "imu";
    if (!std::isfinite(stamp) || stamp<=0 || now_ros-stamp>(lidar?.4:.25) || stamp-now_ros>.1)
      return fail(stream+"_invalid_or_stale_stamp");
    if (previous && (stamp<=previous || stamp-previous>(lidar?.4:.25)))
      return fail(stream+"_time_discontinuity");
    if (lidar && (points<1000 || !std::isfinite(span) || span<.04 || span>.16)) {
      // Driver startup may yield a partial first scan; it cannot seed FAST-LIO.
      if (!lidar_stamp && span>=0 && span<=.16) return false;
      return fail("lidar_invalid_scan_span_or_size");
    }
    previous=stamp; receipt=now_wall; return true;
  }
  bool watchdog(double now_wall) {
    if (lidar_receipt && now_wall-lidar_receipt>.4) return fail("lidar_stream_timeout");
    if (imu_receipt && now_wall-imu_receipt>.25) return fail("imu_stream_timeout");
    return fault.empty();
  }
  bool fail(const std::string& reason) { if (fault.empty()) fault=reason; return false; }
};
}
