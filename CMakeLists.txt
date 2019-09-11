cmake_minimum_required(VERSION 2.8.3)
project(hybrid_servoing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  force_control
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  algorithm/c++
#   MatlabCodeGen/jac_phi_q_block_tilting
#   MatlabCodeGen/jac_phi_q_flip_against_corner
  /usr/include/eigen3
)

find_library(TIMER_LIB TimerLinux HINTS /usr/local/lib/RobotUtilities)
find_library(RUT Utilities HINTS /usr/local/lib/RobotUtilities)

catkin_package()

##
## Build
##

add_library(SOLVE_HFVC algorithm/c++/solvehfvc.cpp algorithm/c++/eiquadprog.hpp)
add_library(RobotBridge experiments/robot_bridge.cpp)

add_subdirectory(experiments/levering_up/cpp_robot_server)
add_subdirectory(experiments/step_crawling/cpp_robot_server)