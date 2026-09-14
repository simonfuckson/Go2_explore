#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace explore {
// GlobalPlanner returns grid centers, including a start that may lie behind
// the measured robot pose. TEB starts at the measured pose instead. Anchor the
// validation path there and check the entire connection to the remaining path.
struct PathSample {
  double x, y, yaw;
};

inline std::vector<PathSample> anchoredPath(
    const std::vector<geometry_msgs::PoseStamped>& path,
    const geometry_msgs::Pose& start, double resolution) {
  if (path.size()<2 || !std::isfinite(resolution) || resolution<=0) return {};
  const double initial_yaw=tf::getYaw(start.orientation);
  if (!std::isfinite(start.position.x) || !std::isfinite(start.position.y) ||
      !std::isfinite(initial_yaw)) return {};
  // Discard only the contiguous, quantized start prefix, bounded by one cell
  // diagonal. Never discard later route points or the goal.
  size_t first=0;
  while (first+1<path.size() && std::hypot(
      path[first].pose.position.x-start.position.x,
      path[first].pose.position.y-start.position.y)<=resolution*std::sqrt(2.)) ++first;
  std::vector<PathSample> output{{start.position.x,start.position.y,initial_yaw}};
  for (size_t i=first;i<path.size();++i) {
    const auto& p=path[i].pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return {};
    const auto previous=output.back();
    double heading=previous.yaw;
    if (i+1<path.size()) {
      const auto& next=path[i+1].pose.position;
      if (!std::isfinite(next.x) || !std::isfinite(next.y)) return {};
      if (std::hypot(next.x-p.x,next.y-p.y)>1e-6)
        heading=std::atan2(next.y-p.y,next.x-p.x);
    }
    const double turn=std::atan2(std::sin(heading-previous.yaw),std::cos(heading-previous.yaw));
    const double distance=std::hypot(p.x-previous.x,p.y-previous.y);
    const int steps=std::max({1,static_cast<int>(std::ceil(distance/(resolution*.5))),
                              static_cast<int>(std::ceil(std::abs(turn)/.05))});
    for (int j=1;j<=steps;++j) {
      const double ratio=static_cast<double>(j)/steps;
      output.push_back({previous.x+(p.x-previous.x)*ratio,
                        previous.y+(p.y-previous.y)*ratio,
                        previous.yaw+turn*ratio});
    }
  }
  return output;
}
}  // namespace explore
