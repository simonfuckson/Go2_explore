#pragma once
#include <algorithm>
#include <cmath>
namespace go2_exploration_safety {
// Occupancy values describe grid cells. Use the same cell-center convention
// as DirectionalStopRegion; looking up arbitrary edge samples includes cells
// outside the rectangle and can falsely trap an already-cleared footprint.
template<class FreeCell>
bool rectangleCellsClear(double x,double y,double yaw,double front,double rear,
                         double half_width,double resolution,double origin_x,
                         double origin_y,FreeCell free_cell) {
  if(!std::isfinite(resolution) || resolution<=0)return false;
  const double c=std::cos(yaw),s=std::sin(yaw);
  double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
  for(double bx:{-rear,front})for(double by:{-half_width,half_width}) {
    const double wx=x+c*bx-s*by,wy=y+s*bx+c*by;
    xmin=std::min(xmin,wx);xmax=std::max(xmax,wx);ymin=std::min(ymin,wy);ymax=std::max(ymax,wy);
  }
  const int x0=std::ceil((xmin-origin_x)/resolution-.5-1e-9),x1=std::floor((xmax-origin_x)/resolution-.5+1e-9);
  const int y0=std::ceil((ymin-origin_y)/resolution-.5-1e-9),y1=std::floor((ymax-origin_y)/resolution-.5+1e-9);
  bool checked=false;
  for(int iy=y0;iy<=y1;++iy)for(int ix=x0;ix<=x1;++ix) {
    const double wx=origin_x+(ix+.5)*resolution,wy=origin_y+(iy+.5)*resolution;
    const double dx=wx-x,dy=wy-y,bx=c*dx+s*dy,by=-s*dx+c*dy;
    if(bx < -rear-1e-9 || bx > front+1e-9 || std::abs(by)>half_width+1e-9)continue;
    checked=true;if(!free_cell(wx,wy))return false;
  }
  return checked;
}
} // namespace go2_exploration_safety
