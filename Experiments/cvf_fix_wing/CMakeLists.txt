cmake_minimum_required(VERSION 3.0.2)
project(cvf_fix_wing)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  visualization_msgs
  nav_msgs
  tf
  message_generation
)
find_package(Eigen3 REQUIRED) 

add_message_files(
  FILES
  dbg_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_runtime
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(track_controller_node
  src/sacontrol_utils.cpp
  src/track_face.cpp
  src/controller_node.cpp)

target_link_libraries(track_controller_node
  ${catkin_LIBRARIES}
)

add_executable(test_sacontrol_node
  src/sacontrol_utils.cpp
  src/track_face.cpp
  src/test_sacontrol.cpp
)

target_link_libraries(test_sacontrol_node
  ${catkin_LIBRARIES}
)

add_executable(bridge_local_world src/bridge_local_world.cpp)

target_link_libraries(bridge_local_world
  ${catkin_LIBRARIES}
)