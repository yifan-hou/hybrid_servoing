include_directories(
  ../../
)

add_library(Tracking2DTaskServer tracking2d.cpp)
target_link_libraries(Tracking2DTaskServer
  RobotBridge
  SOLVE_HFVC
  ${catkin_LIBRARIES}
  ${TIMER_LIB}
  ${RUT}
  ${JAC_LEVERINGUP}
)

add_executable(Tracking2DServerNode server_node.cpp)
target_link_libraries(Tracking2DServerNode
  ${catkin_LIBRARIES}
  Tracking2DTaskServer
  RobotBridge
#   ${TIMER_LIB}
#   ${RUT}
)