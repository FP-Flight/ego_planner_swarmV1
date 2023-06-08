cmake_minimum_required(VERSION 3.0.2)
project(swarm_terminal)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  traj_utils
  bspline_opt
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swarm_terminal
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
CATKIN_DEPENDS traj_utils

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/swarm_terminal.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


add_executable(mission_publish_node src/mission_publish.cpp
src/drone_state.cpp
)

target_link_libraries(mission_publish_node
  ${catkin_LIBRARIES}
)
