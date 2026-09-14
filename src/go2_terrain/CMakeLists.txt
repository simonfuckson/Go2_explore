cmake_minimum_required(VERSION 3.0.2)
project(go2_terrain)

add_compile_options(-std=c++14 -O3 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  diagnostic_msgs
  geometry_msgs
  message_filters
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree segmentation)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES go2_terrain_core go2_terrain_layer
  CATKIN_DEPENDS costmap_2d diagnostic_msgs geometry_msgs message_filters
    pcl_conversions pcl_ros pluginlib roscpp sensor_msgs std_msgs tf2
    tf2_geometry_msgs tf2_ros
  DEPENDS Boost EIGEN3 PCL
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(go2_terrain_core
  src/offline_surface.cpp
  src/export_transaction.cpp
  src/terrain_algorithms.cpp
  src/terrain_grid.cpp)
target_link_libraries(go2_terrain_core
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  yaml-cpp)

add_library(go2_terrain_layer
  src/go2_terrain_layer.cpp)
target_link_libraries(go2_terrain_layer
  go2_terrain_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp)

add_executable(go2_terrain_exporter_node
  src/terrain_exporter.cpp)
target_link_libraries(go2_terrain_exporter_node
  go2_terrain_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(go2_gravity_aligned_cloud_node
  src/gravity_aligned_cloud.cpp)
target_link_libraries(go2_gravity_aligned_cloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(go2_terrain_guard_node
  src/terrain_guard.cpp)
target_link_libraries(go2_terrain_guard_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(go2_offline_surface_test
    test/test_offline_surface.cpp test/gtest_main.cpp)
  if(TARGET go2_offline_surface_test)
    target_link_libraries(go2_offline_surface_test go2_terrain_core ${PCL_LIBRARIES} ${Boost_LIBRARIES})
  endif()
  catkin_add_gtest(go2_terrain_model_test
    test/terrain_model_test.cpp
    test/gtest_main.cpp)
  if(TARGET go2_terrain_model_test)
    target_link_libraries(go2_terrain_model_test ${catkin_LIBRARIES})
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_terrain_algorithms.cpp)
    catkin_add_gtest(go2_terrain_algorithms_test
      test/test_terrain_algorithms.cpp
      test/gtest_main.cpp)
    if(TARGET go2_terrain_algorithms_test)
      target_link_libraries(go2_terrain_algorithms_test
        go2_terrain_core
        ${catkin_LIBRARIES})
    endif()
  endif()
  catkin_add_nosetests(test/test_validate_terrain_map.py)
endif()

install(TARGETS
  go2_terrain_core
  go2_terrain_layer
  go2_terrain_exporter_node
  go2_gravity_aligned_cloud_node
  go2_terrain_guard_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/validate_terrain_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES costmap_plugins.xml LICENSE README.md THIRD_PARTY.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
