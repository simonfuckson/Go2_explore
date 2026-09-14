cmake_minimum_required(VERSION 3.0.2)
project(go2_navigation)

add_compile_options(-std=c++14 -O3 -Wall)
find_package(catkin REQUIRED COMPONENTS
  actionlib actionlib_msgs geometry_msgs move_base_msgs roscpp rospy std_msgs
  std_srvs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(go2_navigation_supervisor_node src/navigation_supervisor.cpp)
target_link_libraries(go2_navigation_supervisor_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/manual_resume.test)
endif()

catkin_install_python(PROGRAMS
  scripts/global_plan_tester.py scripts/analyze_nav_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/nav_log_session.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS go2_navigation_supervisor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
