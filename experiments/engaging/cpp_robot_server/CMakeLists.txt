include_directories(
  ../../
)

add_library(EngagingTaskServer engaging.cpp)
target_link_libraries(EngagingTaskServer
  RobotBridge
  SOLVE_HFVC
  ${catkin_LIBRARIES}
  ${TIMER_LIB}
  ${RUT}
)

add_executable(EngagingServerNode server_node.cpp)
target_link_libraries(EngagingServerNode
  ${catkin_LIBRARIES}
  EngagingTaskServer
  RobotBridge
#   ${TIMER_LIB}
#   ${RUT}
)