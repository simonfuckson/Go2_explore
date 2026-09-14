cmake_minimum_required(VERSION 3.0.2)
project(go2_exploration_safety)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  actionlib_msgs
  diagnostic_msgs
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  rospy
  roscpp
  std_msgs
  sensor_msgs
  std_srvs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS costmap_2d roscpp std_msgs actionlib_msgs diagnostic_msgs geometry_msgs map_msgs
                 move_base_msgs nav_msgs rospy sensor_msgs std_srvs tf2_ros
)

catkin_install_python(
  PROGRAMS scripts/cmd_vel_safety_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_go2_contract.py)
  catkin_add_nosetests(test/test_go2_gate.py)
  catkin_add_nosetests(test/test_gate_cycles.py)
  catkin_add_nosetests(test/test_safety_core.py)
  catkin_add_nosetests(test/test_safety_wiring.py)
endif()
