cmake_minimum_required(VERSION 2.8.3)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  traj_utils
  traj_opt # for traj2odom_node
)

catkin_package(
 CATKIN_DEPENDS traj_utils
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node_udp
  src/bridge_node_udp.cpp 
  )
target_link_libraries(bridge_node_udp 
  ${catkin_LIBRARIES}
  )

add_executable(bridge_node_tcp
  src/bridge_node_tcp.cpp 
  )
target_link_libraries(bridge_node_tcp
  ${catkin_LIBRARIES}
  zmq
  zmqpp
  )

add_executable(traj2odom_node
  src/traj2odom_node.cpp 
  )
target_link_libraries(traj2odom_node 
  ${catkin_LIBRARIES}
  )


