#pragma once
#include <algorithm>
#include <cmath>
#include <go2_exploration_safety/grid_footprint.h>

namespace go2_exploration {
struct PlanarCommand { double v, w; };
struct ShaperLimits {
  double max_v=.30, max_w=.50, walk_floor=.30, turn_floor=.50;
  double deadband_v=.015, deadband_w=.01, sdk_deadband_v=.025;
  double sdk_deadband_w=.01, sdk_turn_floor=.04;
};
inline PlanarCommand sdkEffective(PlanarCommand cmd, const ShaperLimits& p) {
  cmd.v=std::abs(cmd.v)<p.sdk_deadband_v ? 0. : std::max(-p.max_v,std::min(p.max_v,cmd.v));
  cmd.w=std::abs(cmd.w)<p.sdk_deadband_w ? 0. : std::copysign(std::min(p.max_w,std::max(std::abs(cmd.w),p.sdk_turn_floor)),cmd.w);
  return cmd;
}
inline PlanarCommand shapedTarget(PlanarCommand cmd, const ShaperLimits& p) {
  const bool walking=cmd.v>=p.deadband_v;
  cmd.v=walking ? std::min(p.max_v,std::max(cmd.v,p.walk_floor)) : 0.;
  if (std::abs(cmd.w)<p.deadband_w) cmd.w=0.;
  else cmd.w=std::copysign(std::min(p.max_w,std::max(std::abs(cmd.w),walking?0.:p.turn_floor)),cmd.w);
  if (std::abs(cmd.v)<p.sdk_deadband_v) cmd.v=0.;
  if (std::abs(cmd.w)<p.sdk_deadband_w) cmd.w=0.;
  else cmd.w=std::copysign(std::min(p.max_w,std::max(std::abs(cmd.w),p.sdk_turn_floor)),cmd.w);
  return cmd;
}
template<class Clear>
bool transitionClear(PlanarCommand current, PlanarCommand target,const ShaperLimits& p,Clear clear) {
  // Linear acceleration, yaw reversal hold and pure-turn stepping are separate
  // in the preserved shaper. Checking only their endpoints misses tighter arcs
  // while starting. Cover the rectangle of intermediate speeds, with SDK floors.
  const int nv=std::max(1,int(std::ceil(std::abs(target.v-current.v)/.025)));
  const int nw=std::max(1,int(std::ceil(std::abs(target.w-current.w)/.04)));
  for (int i=0;i<=nv;++i)for(int j=0;j<=nw;++j) {
    PlanarCommand cmd{current.v+(target.v-current.v)*i/nv,current.w+(target.w-current.w)*j/nw};
    if (!clear(sdkEffective(cmd,p))) return false;
  }
  return true;
}
struct SweepGeometry {
  double front=.35,rear=.35,half_width=.155,margin=.03,forward_clearance=.18;
  double reaction=.20,decel_v=.20,decel_w=.40,horizon=1.50;
  double origin_x=0,origin_y=0,width=0,height=0;
};
template<class FreeCell>
bool sweepClear(double robot_x,double robot_y,double yaw,PlanarCommand cmd,
                const SweepGeometry& p,double resolution,FreeCell free_cell) {
  const double duration=std::min(p.horizon,p.reaction+std::max(std::abs(cmd.v)/p.decel_v,std::abs(cmd.w)/p.decel_w));
  const double c=std::cos(yaw),s=std::sin(yaw);
  for(int i=0;i<=12;++i) {
    const double t=duration*i/12,a=cmd.w*t;
    const double px=std::abs(cmd.w)<1e-6?cmd.v*t:cmd.v/cmd.w*std::sin(a);
    const double py=std::abs(cmd.w)<1e-6?0:cmd.v/cmd.w*(1-std::cos(a));
    const double ca=std::cos(yaw+a),sa=std::sin(yaw+a);
    const double end=(i==0 && cmd.v>1e-4)?p.front+std::max(p.forward_clearance,cmd.v*p.reaction+cmd.v*cmd.v/(2*p.decel_v)):p.front+p.margin;
    const double cx=robot_x+c*px-s*py,cy=robot_y+s*px+c*py;
    if(p.width>0 && p.height>0)for(double x:{-p.rear-p.margin,end})for(double y:{-p.half_width-p.margin,p.half_width+p.margin}) {
      const double wx=cx+ca*x-sa*y,wy=cy+sa*x+ca*y;
      if(wx<p.origin_x || wy<p.origin_y || wx>=p.origin_x+p.width || wy>=p.origin_y+p.height)return false;
    }
    if(!go2_exploration_safety::rectangleCellsClear(cx,cy,yaw+a,end,p.rear+p.margin,p.half_width+p.margin,
          resolution,p.origin_x,p.origin_y,free_cell))return false;
  }
  return true;
}
} // namespace go2_exploration
