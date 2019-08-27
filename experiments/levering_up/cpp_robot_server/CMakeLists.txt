include_directories(
  ../../../examples/levering_up/generated/codegen/dll/jacobian_levering_up
  ../../
)

find_library(JAC_LEVERINGUP NAMES libjacobian_levering_up.so
	HINTS ../../../examples/levering_up/generated/codegen/dll/jacobian_levering_up/
)

add_library(LeveringUpTaskServer levering_up.cpp)
target_link_libraries(LeveringUpTaskServer
  RobotBridge
  SOLVE_HFVC
  ${catkin_LIBRARIES}
  ${TIMER_LIB}
  ${RUT}
  ${JAC_LEVERINGUP}
)

add_executable(LeveringUpServerNode server_node.cpp)
target_link_libraries(LeveringUpServerNode
  ${catkin_LIBRARIES}
  LeveringUpTaskServer
  RobotBridge
#   ${TIMER_LIB}
#   ${RUT}
)