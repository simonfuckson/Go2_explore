// Export the existing obstacle layer BEFORE inflation. No second cloud
// classifier or ray tracer: local control and exploration use the same evidence.
#include <costmap_2d/obstacle_layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <stdexcept>

namespace go2_exploration {
class ObservedObstacleLayer : public costmap_2d::ObstacleLayer {
 public:
  void onInitialize() override {
    costmap_2d::ObstacleLayer::onInitialize();
    ros::NodeHandle nh("~/" + name_);
    std::string topic;
    nh.param<std::string>("observation_map_topic", topic,
                          "/exploration/observed_local_map");
    publisher_ = nh.advertise<nav_msgs::OccupancyGrid>(topic, 1, true);
    blind_pub_ = nh.advertise<visualization_msgs::Marker>("/exploration/rear_blind_zone", 1, true);
    load_time_ = ros::Time::now();
    if (!ros::param::get("/go2_terrain_guard/sensor_height", sensor_height_))
      throw std::runtime_error("GO2 terrain sensor height is missing");
  }

  void updateCosts(costmap_2d::Costmap2D& master, int x0, int y0,
                   int x1, int y1) override {
    costmap_2d::ObstacleLayer::updateCosts(master, x0, y0, x1, y1);
    const auto now = ros::Time::now();
    if (!enabled_ || !isCurrent() || (now-last_publish_).toSec() < .5) return;
    last_publish_ = now;
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = now;
    grid.header.frame_id = layered_costmap_->getGlobalFrameID();
    grid.info.map_load_time = load_time_;
    grid.info.resolution = getResolution();
    grid.info.width = getSizeInCellsX();
    grid.info.height = getSizeInCellsY();
    grid.info.origin.position.x = getOriginX();
    grid.info.origin.position.y = getOriginY();
    grid.info.origin.orientation.w = 1;
    grid.data.resize(grid.info.width * grid.info.height);
    const auto* cells = getCharMap();
    for (size_t i=0; i<grid.data.size(); ++i)
      grid.data[i] = cells[i] == costmap_2d::FREE_SPACE ? 0 :
          cells[i] == costmap_2d::LETHAL_OBSTACLE ? 100 : -1;
    publisher_.publish(grid);
    // Nominal ground-view mask used by goal scoring, not a costmap obstacle.
    visualization_msgs::Marker blind;
    blind.header.stamp = now; blind.header.frame_id = "terrain_sensor";
    blind.ns = "rear_ground_blind"; blind.id = 0;
    blind.type = visualization_msgs::Marker::LINE_STRIP;
    blind.action = visualization_msgs::Marker::ADD;
    blind.pose.orientation.w = 1; blind.scale.x = .035;
    blind.color.r = 1; blind.color.g = .5; blind.color.a = .8;
    for (const auto& xy : std::vector<std::pair<double,double>>{{0,-3},{-3,-3},{-3,3},{0,3},{0,-3}}) {
      geometry_msgs::Point p; p.x = xy.first; p.y = xy.second; p.z = -sensor_height_;
      blind.points.push_back(p);
    }
    blind_pub_.publish(blind);
  }
 private:
  ros::Publisher publisher_;
  ros::Publisher blind_pub_;
  ros::Time last_publish_, load_time_;
  double sensor_height_=0;
};
}
PLUGINLIB_EXPORT_CLASS(go2_exploration::ObservedObstacleLayer, costmap_2d::Layer)
