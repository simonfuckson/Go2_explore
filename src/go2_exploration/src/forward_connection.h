#pragma once
#include <cmath>
#include <limits>
#include <vector>
#include "command_envelope.h"

namespace go2_exploration {
struct RoutePoint { double x,y; };
struct RouteProjection { double distance=std::numeric_limits<double>::infinity(),progress=0; };
inline RouteProjection projectRoute(const std::vector<RoutePoint>& route,double x,double y) {
  RouteProjection result;double length=0;
  for(size_t i=1;i<route.size();++i) {
    const double dx=route[i].x-route[i-1].x,dy=route[i].y-route[i-1].y,len=std::hypot(dx,dy);
    if(len<1e-6)continue;
    const double t=std::max(0.,std::min(1.,((x-route[i-1].x)*dx+(y-route[i-1].y)*dy)/(len*len)));
    const double distance=std::hypot(x-route[i-1].x-t*dx,y-route[i-1].y-t*dy);
    if(distance<result.distance)result={distance,length+t*len};
    length+=len;
  }
  return result;
}
inline RoutePoint arcPoint(PlanarCommand cmd,double t) {
  return std::abs(cmd.w)<1e-6 ? RoutePoint{cmd.v*t,0.} :
    RoutePoint{cmd.v/cmd.w*std::sin(cmd.w*t),cmd.v/cmd.w*(1-std::cos(cmd.w*t))};
}
// Bounded forward connections to the current route. A blocked turn never
// authorizes a blind movement: every candidate needs the complete stop sweep.
template<class SafeCommand>
bool forwardConnection(const std::vector<RoutePoint>& route,const ShaperLimits& limits,
                       SafeCommand safe,PlanarCommand& selected) {
  if(route.size()<2 || std::hypot(route.back().x,route.back().y)<=.35)return false;
  const auto initial=projectRoute(route,0,0);
  if(initial.distance>.15)return false;
  double best=std::numeric_limits<double>::infinity();bool found=false;
  for(const double w:{0.,-.10,.10,-.20,.20,-.30,.30}) {
    const auto cmd=shapedTarget({limits.max_v,w},limits);
    if(cmd.v<=0 || std::abs(cmd.w)>limits.max_w)continue;
    bool follows=true;RouteProjection end;
    for(int i=1;i<=8;++i) {
      const auto p=arcPoint(cmd,.1*i);end=projectRoute(route,p.x,p.y);
      if(end.distance>.25 || end.progress+0.025<initial.progress) {follows=false;break;}
    }
    const double progress=end.progress-initial.progress;
    if(!follows || progress<.05 || !safe(cmd))continue;
    const double score=3*end.distance-progress+.10*std::abs(cmd.w);
    if(score<best) {best=score;selected=cmd;found=true;}
  }
  return found;
}
} // namespace go2_exploration
