#pragma once
#include <algorithm>
#include <cmath>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

namespace explore {
inline double angleDifference(double a, double b) {
  return std::atan2(std::sin(a-b),std::cos(a-b));
}
// Nominal ground visibility for scoring only; never used to clear a cell.
// Collision mapping still uses the established real-return ray projection.
inline bool groundInView(double forward, double lateral, double range,
                         double sensor_height, double pitch,
                         double sensor_x, double sensor_y, double sensor_yaw, double sensor_roll) {
  forward-=sensor_x; lateral-=sensor_y;
  const double f=std::cos(sensor_yaw)*forward+std::sin(sensor_yaw)*lateral;
  lateral=-std::sin(sensor_yaw)*forward+std::cos(sensor_yaw)*lateral;
  forward=f;
  if (forward<0 || std::hypot(forward,lateral)>range || std::hypot(forward,lateral)<.35) return false;
  const double sx=std::cos(pitch)*forward+std::sin(pitch)*sensor_height;
  const double sz=std::sin(pitch)*forward-std::cos(pitch)*sensor_height;
  const double sy=std::cos(sensor_roll)*lateral+std::sin(sensor_roll)*sz;
  const double rz=-std::sin(sensor_roll)*lateral+std::cos(sensor_roll)*sz;
  const double elevation=std::atan2(rz,std::hypot(sx,sy));
  return elevation>=-7.*M_PI/180. && elevation<=52.*M_PI/180.;
}
inline double expectedGroundGain(const costmap_2d::Costmap2D& map,
                                 double x,double y,double yaw,double range,
                                 double height,double pitch,
                                 double sensor_x,double sensor_y,double sensor_yaw,double sensor_roll,
                                 const costmap_2d::Costmap2D* knowledge=nullptr) {
  if (!knowledge) knowledge=&map;
  const double resolution=map.getResolution(), step=std::max(.15,resolution);
  const double c=std::cos(yaw),s=std::sin(yaw);
  double area=0;
  for (double dx=-range;dx<=range;dx+=step) for (double dy=-range;dy<=range;dy+=step) {
    if (!groundInView(c*dx+s*dy,-s*dx+c*dy,range,height,pitch,sensor_x,sensor_y,sensor_yaw,sensor_roll)) continue;
    unsigned mx,my;
    if (!knowledge->worldToMap(x+dx,y+dy,mx,my) || knowledge->getCost(mx,my)!=costmap_2d::NO_INFORMATION) continue;
    const double sx=x+sensor_x*c-sensor_y*s,sy=y+sensor_x*s+sensor_y*c;
    const int n=std::max(1,static_cast<int>(std::ceil(std::hypot(x+dx-sx,y+dy-sy)/resolution)));
    bool occluded=false;
    for (int i=1;i<n;++i) {
      unsigned ix,iy;
      if (!map.worldToMap(sx+(x+dx-sx)*i/n,sy+(y+dy-sy)*i/n,ix,iy)
          || map.getCost(ix,iy)==costmap_2d::LETHAL_OBSTACLE) {occluded=true;break;}
    }
    if (!occluded) area+=step*step;
  }
  return area;
}
} // namespace explore
