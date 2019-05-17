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
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  include
  MatlabCodeGen/jac_phi_q_block_tilting
  MatlabCodeGen/jac_phi_q_flip_against_corner
  /usr/include/eigen3
)

find_library(TIMER_LIB TimerLinux HINTS /usr/local/lib/yifanlibrary)
find_library(UT Utilities HINTS /usr/local/lib/yifanlibrary)


add_library(SOLVE_HFVC algorithms/solvehfvc.cpp algorithms/eiquadprog.hpp)

add_library(Plane_Engaging apps/plane_engaging.cpp)
target_link_libraries(Plane_Engaging
  SOLVE_HFVC
  ${TIMER_LIB}
  ${UT}
#   ${CMAKE_SOURCE_DIR}/force_control_bridge_ros/MatlabCodeGen/jac_phi_q_block_tilting/jac_phi_q_block_tilting.a
#   ${CMAKE_SOURCE_DIR}/force_control_bridge_ros/MatlabCodeGen/jac_phi_q_flip_against_corner/jac_phi_q_flip_against_corner.a
)

add_executable(${PROJECT_NAME}_node src/robot_bridge.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Plane_Engaging
)
