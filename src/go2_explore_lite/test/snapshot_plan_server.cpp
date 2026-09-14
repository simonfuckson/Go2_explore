// Offline-only service for reproducing selection against a recorded costmap.
// There are no command publishers, sensor drivers, or SDK dependencies.
#include <ros/ros.h>
#include <ros/serialization.h>
#include <global_planner/planner_core.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <iostream>

class SnapshotPlanner {
 public:
  explicit SnapshotPlanner(const std::string& file) {
    std::ifstream input(file,std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)),{});
    if(bytes.empty())throw std::runtime_error("Empty snapshot map");
    nav_msgs::OccupancyGrid map;
    ros::serialization::IStream stream(bytes.data(),bytes.size());
    ros::serialization::deserialize(stream,map);
    if(map.data.size()!=map.info.width*map.info.height||map.info.resolution<=0)
      throw std::runtime_error("Invalid snapshot map");
    map_.resizeMap(map.info.width,map.info.height,map.info.resolution,
                   map.info.origin.position.x,map.info.origin.position.y);
    for(size_t i=0;i<map.data.size();++i) {
      const int c=map.data[i];
      map_.getCharMap()[i]=c<0?255:c==100?254:c==99?253:c==0?0:1+(251*(c-1))/97;
    }
    planner_.initialize("GlobalPlanner",&map_,map.header.frame_id);
    service_=nh_.advertiseService("/move_base/make_plan",&SnapshotPlanner::plan,this);
  }
 private:
  bool plan(nav_msgs::GetPlan::Request& req,nav_msgs::GetPlan::Response& res) {
    std::cout<<"GOAL "<<req.goal.pose.position.x<<" "<<req.goal.pose.position.y<<std::endl;
    return planner_.makePlanService(req,res);
  }
  ros::NodeHandle nh_;
  costmap_2d::Costmap2D map_;
  global_planner::GlobalPlanner planner_;
  ros::ServiceServer service_;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"snapshot_planner");
  if(argc!=2)return 2;
  SnapshotPlanner planner(argv[1]);ros::spin();return 0;
}
