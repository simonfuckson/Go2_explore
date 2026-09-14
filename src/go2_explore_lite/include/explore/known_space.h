#pragma once
#include <algorithm>
#include <cmath>
#include <set>
#include <vector>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <go2_exploration_safety/grid_footprint.h>

namespace explore {
// Check actual occupancy with the physical rectangle, not inflated center
// costs applied to the rectangle a second time. Unknown/outside stay invalid.
inline bool knownFootprint(const costmap_2d::Costmap2D& map, double x, double y,
                           double yaw, double front, double rear,
                           double half_width, geometry_msgs::Point* blocked=nullptr,
                           unsigned char* blocked_cost=nullptr) {
  const double c=std::cos(yaw), s=std::sin(yaw);
  for(double bx:{-rear,front})for(double by:{-half_width,half_width}) {
    unsigned mx,my;
    const double wx=x+c*bx-s*by,wy=y+s*bx+c*by;
    if(!map.worldToMap(wx,wy,mx,my)) {
      if(blocked) {blocked->x=wx;blocked->y=wy;}
      if(blocked_cost)*blocked_cost=costmap_2d::NO_INFORMATION;
      return false;
    }
  }
  return go2_exploration_safety::rectangleCellsClear(x,y,yaw,front,rear,half_width,
    map.getResolution(),map.getOriginX(),map.getOriginY(),[&](double wx,double wy) {
      unsigned mx,my;
      const bool inside=map.worldToMap(wx,wy,mx,my);
      const unsigned char cost=inside?map.getCost(mx,my):costmap_2d::NO_INFORMATION;
      if (cost>=costmap_2d::LETHAL_OBSTACLE) {
        if (blocked) {blocked->x=wx;blocked->y=wy;}
        if (blocked_cost) *blocked_cost=cost;
        return false;
      }
      return true;
    });
}

inline std::vector<geometry_msgs::Point> knownApproaches(
    const costmap_2d::Costmap2D& map,
    const std::vector<geometry_msgs::Point>& frontier, double radius=.75) {
  std::set<unsigned> cells;
  const int n=std::ceil(radius/map.getResolution());
  for (const auto& p:frontier) {
    unsigned fx,fy;
    if (!map.worldToMap(p.x,p.y,fx,fy)) continue;
    for (int dy=-n;dy<=n;++dy) for (int dx=-n;dx<=n;++dx) {
      const int x=static_cast<int>(fx)+dx,y=static_cast<int>(fy)+dy;
      if (x<0 || y<0 || x>=static_cast<int>(map.getSizeInCellsX())
          || y>=static_cast<int>(map.getSizeInCellsY())
          || std::hypot(dx,dy)*map.getResolution()>radius) continue;
      if (map.getCost(x,y)<costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        cells.insert(map.getIndex(x,y));
    }
  }
  std::vector<geometry_msgs::Point> result;
  for (const auto index:cells) {
    unsigned x,y;
    map.indexToCells(index,x,y);
    geometry_msgs::Point p;
    map.mapToWorld(x,y,p.x,p.y);
    result.push_back(p);
  }
  return result;
}
} // namespace explore
