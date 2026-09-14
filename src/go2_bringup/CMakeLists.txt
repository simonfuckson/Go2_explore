cmake_minimum_required(VERSION 3.0.2)
project(go2_bringup)

add_compile_options(-std=c++14 -O3 -Wall)
find_package(catkin REQUIRED COMPONENTS livox_ros_driver2 nav_msgs roscpp sensor_msgs std_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_executable(go2_system_monitor_node src/system_monitor.cpp)
add_dependencies(go2_system_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(go2_system_monitor_node ${catkin_LIBRARIES})
install(TARGETS go2_system_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
