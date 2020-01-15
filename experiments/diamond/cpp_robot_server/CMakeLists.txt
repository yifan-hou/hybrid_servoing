include_directories(
  ../../
)

add_library(DiamondTaskServer diamond.cpp)
target_link_libraries(DiamondTaskServer
  RobotBridge
  SOLVE_HFVC
  ${catkin_LIBRARIES}
  ${TIMER_LIB}
  ${RUT}
)

add_executable(DiamondServerNode server_node.cpp)
target_link_libraries(DiamondServerNode
  ${catkin_LIBRARIES}
  DiamondTaskServer
  RobotBridge
#   ${TIMER_LIB}
#   ${RUT}
)