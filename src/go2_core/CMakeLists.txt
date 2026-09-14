cmake_minimum_required(VERSION 3.0.2)
project(go2_core)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  tf2_ros
  xmlrpcpp
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(go2_pose_adapter_node src/pose_adapter.cpp)
target_link_libraries(go2_pose_adapter_node ${catkin_LIBRARIES})

add_executable(go2_cloud_adapter_node src/cloud_adapter.cpp)
target_link_libraries(go2_cloud_adapter_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/tf_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS go2_pose_adapter_node go2_cloud_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
