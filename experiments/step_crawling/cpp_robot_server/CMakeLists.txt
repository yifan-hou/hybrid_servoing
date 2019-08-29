include_directories(
  ../../
)

add_library(StepCrawlingTaskServer step_crawling.cpp)
target_link_libraries(StepCrawlingTaskServer
  RobotBridge
  SOLVE_HFVC
  ${catkin_LIBRARIES}
  ${TIMER_LIB}
  ${RUT}
)

add_executable(StepCrawlingServerNode server_node.cpp)
target_link_libraries(StepCrawlingServerNode
  ${catkin_LIBRARIES}
  StepCrawlingTaskServer
  RobotBridge
#   ${TIMER_LIB}
#   ${RUT}
)