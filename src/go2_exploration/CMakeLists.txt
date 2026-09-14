cmake_minimum_required(VERSION 3.0.2)
project(go2_exploration)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  go2_exploration_safety livox_ros_driver2
  costmap_2d diagnostic_msgs geometry_msgs nav_msgs visualization_msgs
  pluginlib roscpp rospy sensor_msgs std_msgs std_srvs tf2
  tf2_ros tf2_geometry_msgs nav_core teb_local_planner rviz)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  LIBRARIES go2_exploration_plugins
  CATKIN_DEPENDS costmap_2d diagnostic_msgs geometry_msgs
                 nav_msgs pluginlib roscpp rospy sensor_msgs std_msgs
                 std_srvs tf2 tf2_ros tf2_geometry_msgs nav_core teb_local_planner)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(go2_exploration_plugins src/forward_teb_planner.cpp src/observed_obstacle_layer.cpp)
target_link_libraries(go2_exploration_plugins ${catkin_LIBRARIES})
add_library(go2_exploration_rviz src/status_panel.cpp)
target_link_libraries(go2_exploration_rviz ${catkin_LIBRARIES} Qt5::Widgets)
add_executable(go2_exploration_sensor_guard src/sensor_guard.cpp)
add_dependencies(go2_exploration_sensor_guard ${catkin_EXPORTED_TARGETS})
target_link_libraries(go2_exploration_sensor_guard ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_command_envelope test/test_command_envelope.cpp)
  catkin_add_gtest(test_sensor_continuity test/test_sensor_continuity.cpp)
  catkin_add_gtest(test_self_filter_geometry test/test_self_filter_geometry.cpp)
  catkin_add_nosetests(test/test_supervisor_start.py)
  catkin_add_nosetests(test/test_observed_map_memory.py)
  catkin_add_nosetests(test/test_dashboard.py)
  catkin_add_nosetests(test/test_ground_clearing.py)
endif()

install(TARGETS go2_exploration_plugins go2_exploration_rviz go2_exploration_sensor_guard
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES go2_exploration_plugins.xml rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS test/synthetic_world.py scripts/ground_clearing_adapter.py scripts/odom_health.py scripts/explore_supervisor.py scripts/check_exploration.py scripts/check_navigation_clearance.py
                 scripts/frontier_costmap_adapter.py scripts/observed_map_memory.py scripts/exploration_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
