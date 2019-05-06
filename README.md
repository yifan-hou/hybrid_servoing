A framework for testing forcecontrol tasks/algorithms. Serve as a bridge between low level forcecontrol and high level Matlab GUI commands.

# Key components
robot_bridge.cpp: A server that communicates with Matlab GUI by ros service and file IO. Provides services for motion primitives and tasks. Call forcecontrol internally.

hybrid_servoing_tasks: One task class for each specific task.

solvehfvc: C++ implementation of hybrid servoing algorithm.
eiquadprog.hpp: C++ implementation of quadratic programming.
