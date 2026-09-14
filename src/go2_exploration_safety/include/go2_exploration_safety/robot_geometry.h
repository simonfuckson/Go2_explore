#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <ros/ros.h>
#include <costmap_2d/footprint.h>

namespace go2_exploration_safety {
struct RobotGeometry {
  double front, rear, half_width, margin, clearance;
  static RobotGeometry load() {
    XmlRpc::XmlRpcValue footprint;
    if (!ros::param::get("/move_base/global_costmap/footprint", footprint))
      throw std::runtime_error("GO2 footprint is missing");
    // Costmap2DROS normalizes this parameter from a YAML array to a string.
    std::vector<geometry_msgs::Point> points;
    if (footprint.getType() == XmlRpc::XmlRpcValue::TypeString) {
      if (!costmap_2d::makeFootprintFromString(static_cast<std::string>(footprint), points))
        throw std::runtime_error("Invalid GO2 footprint string");
    } else {
      points=costmap_2d::makeFootprintFromXMLRPC(footprint,"/move_base/global_costmap/footprint");
    }
    if (points.size()<3) throw std::runtime_error("Invalid GO2 footprint polygon");
    RobotGeometry g{0, 0, 0, 0, 0};
    for (const auto& point : points) {
      const double x=point.x;
      const double y=point.y;
      if (!std::isfinite(x) || !std::isfinite(y))
        throw std::runtime_error("Invalid GO2 footprint");
      g.front=std::max(g.front,x); g.rear=std::max(g.rear,-x);
      g.half_width=std::max(g.half_width,std::abs(y));
    }
    if (!ros::param::get("/move_base/global_costmap/footprint_padding",g.margin) ||
        !std::isfinite(g.margin) || g.margin<0 || g.front<=0 || g.rear<=0 || g.half_width<=0)
      throw std::runtime_error("Invalid GO2 footprint padding/envelope");
    double min_distance, reaction, deceleration, speed;
    ros::param::param("/go2_exploration_safety/min_forward_clearance",min_distance,.18);
    ros::param::param("/go2_exploration_safety/reaction_time",reaction,.20);
    ros::param::param("/go2_exploration_safety/linear_deceleration",deceleration,.20);
    ros::param::param("/go2_exploration_safety/max_linear_speed",speed,.30);
    if (deceleration<=0 || speed<0) throw std::runtime_error("Invalid braking limits");
    g.clearance=std::max(min_distance,speed*reaction+speed*speed/(2*deceleration));
    return g;
  }
};
}
