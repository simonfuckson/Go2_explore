cmake_minimum_required(VERSION 3.0.2)
project(go2_localization)

add_compile_options(-std=c++14 -O3 -Wall)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_filters nav_msgs pcl_conversions pcl_ros roscpp
  sensor_msgs std_msgs tf tf2_ros tf_conversions)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(go2_ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp)
target_link_libraries(go2_ndt_omp ${PCL_LIBRARIES})

add_executable(go2_ndt_localizer_node src/ndt_localizer.cpp)
target_link_libraries(go2_ndt_localizer_node
  go2_ndt_omp ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(go2_map_loader_node src/map_loader.cpp)
target_link_libraries(go2_map_loader_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(go2_localization_guard_node src/localization_guard.cpp)
target_link_libraries(go2_localization_guard_node ${catkin_LIBRARIES})

install(TARGETS go2_ndt_omp go2_ndt_localizer_node go2_map_loader_node
                go2_localization_guard_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
