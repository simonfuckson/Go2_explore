cmake_minimum_required(VERSION 3.0.2)
project(go2_mapping)

add_compile_options(-std=c++14 -O3 -Wall)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_srvs)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_srvs
  DEPENDS PCL Boost EIGEN3)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable(go2_map_builder_node src/map_builder.cpp)
target_link_libraries(go2_map_builder_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  ${Boost_LIBRARIES} OpenSSL::Crypto Threads::Threads)

add_executable(go2_map_exporter_node src/occupancy_map_exporter.cpp)
target_link_libraries(go2_map_exporter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(dynamic_mapping_test
    test/dynamic_mapping_test.cpp
    test/gtest_main.cpp)
  if(TARGET dynamic_mapping_test)
    target_link_libraries(dynamic_mapping_test
      ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES}
      OpenSSL::Crypto Threads::Threads)
  endif()
endif()

install(TARGETS go2_map_builder_node go2_map_exporter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
