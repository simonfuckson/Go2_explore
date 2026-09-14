cmake_minimum_required(VERSION 3.0.2)
project(go2_control)

add_compile_options(-std=c++14 -O3 -Wall)
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  std_srvs)

add_message_files(FILES
  Go2MotorState.msg
  Go2BmsState.msg
  Go2PathPoint.msg
  Go2LowState.msg
  Go2SportState.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  std_srvs)

set(UNITREE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Unitree_SDK2"
  CACHE PATH "Path to the Unitree SDK2 source tree")
if(NOT EXISTS "${UNITREE_SDK_ROOT}/include/unitree/robot/go2/sport/sport_client.hpp")
  message(FATAL_ERROR "Unitree SDK2 not found at ${UNITREE_SDK_ROOT}")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UNITREE_SDK_ROOT}/include
  ${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx
  ${UNITREE_SDK_ROOT}/thirdparty/include)

add_executable(go2_velocity_shaper_node src/velocity_shaper.cpp)
add_dependencies(go2_velocity_shaper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go2_velocity_shaper_node ${catkin_LIBRARIES})

add_executable(go2_sdk_bridge_mock_node src/sdk_bridge_mock.cpp)
add_dependencies(go2_sdk_bridge_mock_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go2_sdk_bridge_mock_node ${catkin_LIBRARIES})

add_executable(go2_sdk_bridge_real_node src/sdk_bridge.cpp)
add_dependencies(go2_sdk_bridge_real_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go2_sdk_bridge_real_node
  ${catkin_LIBRARIES}
  ${UNITREE_SDK_ROOT}/lib/aarch64/libunitree_sdk2.a
  ${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64/libddscxx.so
  ${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64/libddsc.so
  pthread dl rt)

add_executable(go2_sdk_official_motion_test_node
  src/sdk_official_motion_test.cpp)
add_dependencies(go2_sdk_official_motion_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go2_sdk_official_motion_test_node
  ${catkin_LIBRARIES}
  ${UNITREE_SDK_ROOT}/lib/aarch64/libunitree_sdk2.a
  ${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64/libddscxx.so
  ${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64/libddsc.so
  pthread dl rt)

install(TARGETS go2_velocity_shaper_node go2_sdk_bridge_mock_node
                go2_sdk_bridge_real_node go2_sdk_official_motion_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(go2_classic_gait_test test/classic_gait_test.cpp)
  if(TARGET go2_classic_gait_test)
    target_link_libraries(go2_classic_gait_test ${catkin_LIBRARIES})
  endif()
endif()
