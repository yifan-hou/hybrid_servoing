cmake_minimum_required(VERSION 2.8.3)
project(force_control_bridge_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  forcecontrol
  netft_rdt_driver
  egm
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES regrasping
#  CATKIN_DEPENDS forcecontrol roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  MatlabCodeGen/jac_phi_q_block_tilting
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  ~/Git/ZZJHand/include/
  ~/Git/cpplibrary/include/
  # ${Boost_INCLUDE_DIRS}
)

add_library(SOLVE_HFVC src/solvehfvc.cpp include/eiquadprog.hpp)

add_library(HS_tasks src/hybrid_servoing_tasks.cpp)
target_link_libraries(HS_tasks
  SOLVE_HFVC
  /usr0/home/yifanh/Git/ZZJHand/build/libHand.so
  /usr0/home/yifanh/Git/cpplibrary/build/libTimerLinux.so
  ${CMAKE_SOURCE_DIR}/force_control_bridge_ros/MatlabCodeGen/jac_phi_q_block_tilting/jac_phi_q_block_tilting.a
)


add_executable(${PROJECT_NAME}_node src/robot_bridge.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  HS_tasks
  /usr0/home/yifanh/Git/ZZJHand/build/libHand.so
)
