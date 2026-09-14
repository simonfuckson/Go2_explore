#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <geometry_msgs/Point.h>
#include <explore/view_gain.h>

namespace explore {
using CandidateKey=std::pair<int,int>;
inline CandidateKey candidateKey(double x,double y,double resolution) {
  // World coordinates remain stable when the online map grows at its origin.
  return {static_cast<int>(std::floor(x/resolution)),static_cast<int>(std::floor(y/resolution))};
}

// Try one point per 20 cm region first, then refine every region. Previously
// the first point permanently erased all other 5 cm positions in that region.
inline std::vector<geometry_msgs::Point> diverseCandidates(
    const std::vector<geometry_msgs::Point>& ranked,double resolution) {
  std::set<CandidateKey> seen;
  std::map<CandidateKey,size_t> indices;
  std::vector<std::vector<geometry_msgs::Point>> groups;
  size_t largest=0;
  for(const auto& p:ranked) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y))continue;
    if(!seen.insert(candidateKey(p.x,p.y,resolution)).second)continue;
    const auto key=candidateKey(p.x,p.y,std::max(.20,resolution));
    auto inserted=indices.emplace(key,groups.size());
    if(inserted.second)groups.emplace_back();
    auto& group=groups[inserted.first->second];group.push_back(p);
    largest=std::max(largest,group.size());
  }
  std::vector<geometry_msgs::Point> out;out.reserve(seen.size());
  for(size_t depth=0;depth<largest;++depth)
    for(const auto& group:groups)if(depth<group.size())out.push_back(group[depth]);
  return out;
}

// Rotate the bounded search between timer ticks. Expiring cache entries at
// the head must not starve all the remaining candidates indefinitely.
struct CandidateCursor {
  size_t next=0,visited=0,size=0;
  CandidateCursor(size_t offset,size_t count):next(count?offset%count:0),size(count){}
  bool empty()const{return visited>=size;}
  size_t pop(){const auto result=next;next=(next+1)%size;++visited;return result;}
};

inline bool retryPoseChanged(double x,double y,double yaw,
                             double previous_x,double previous_y,double previous_yaw) {
  return std::hypot(x-previous_x,y-previous_y)>.10 ||
         std::abs(angleDifference(yaw,previous_yaw))>.15;
}

// Prefer information per estimated travel second. Translation and rotation
// are additive, a conservative estimate for a forward-only robot.
inline double informationRate(double gain,double length,double turn,
                              double speed,double yaw_speed,double settle_seconds) {
  if(!std::isfinite(gain)||!std::isfinite(length)||!std::isfinite(turn)||
     !std::isfinite(speed)||!std::isfinite(yaw_speed)||!std::isfinite(settle_seconds)||
     gain<0||length<0||turn<0||speed<=0||yaw_speed<=0||settle_seconds<=0)return 0;
  return gain/(length/speed+turn/yaw_speed+settle_seconds);
}
} // namespace explore
