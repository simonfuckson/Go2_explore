cmake_minimum_required(VERSION 3.0.2)
project(patchworkpp)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  jsk_recognition_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(PCL 1.7 REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    jsk_recognition_msgs pcl_conversions pcl_ros roscpp sensor_msgs tf
  DEPENDS Boost EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Robot 1 only needs the online ROS wrapper. Upstream demos and the embedded
# JSK message copy remain vendored for license/source traceability but are not
# compiled; the system ros-noetic-jsk-recognition-msgs package is authoritative.
add_executable(go2_patchwork_wrapper_node src/go2_patchwork_wrapper.cpp)
target_link_libraries(go2_patchwork_wrapper_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS go2_patchwork_wrapper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES LICENSE LOCAL_CHANGES.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
