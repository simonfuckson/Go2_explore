#pragma once
#include <cmath>
#include <vector>
#include <explore/view_gain.h>
namespace explore {
// Rank only reachable viewing headings. A high-gain impossible turn must not
// discard another feasible view at exactly the same destination.
template<class ClearFootprint,class Gain>
bool feasibleArrivalView(const std::vector<double>& headings,double arrival,
                         ClearFootprint clear,Gain gain,double& yaw,double& value) {
  value=-1;
  for(const double heading:headings) {
    const double turn=angleDifference(heading,arrival);
    const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(turn)/.05)));
    bool reachable=true;
    for(int i=0;i<=steps;++i)if(!clear(arrival+turn*i/steps)) {reachable=false;break;}
    if(!reachable)continue;
    const double candidate=gain(heading);
    if(candidate>=.01 && candidate>value) {value=candidate;yaw=heading;}
  }
  return value>=.01;
}
} // namespace explore
