# hybrid_servoing
A package for computing hybrid force-velocity control actions to execute a contact-rich motion plan robustly.

matlab implementation: `algorithm/matlab/solvehfvc.m`.
c++ implementation: `algorithm/c++/solvehfvc.h`.

Author: Yifan Hou
yifanh at cmu dot edu

**Reference**
Yifan Hou, and Matthew T. Mason, " Robust Execution of Contact-Rich Motion Plans by Hybrid Force-Velocity Control“, IEEE International Conference on Robotics and Automation (ICRA) 2019.

# Install
## Dependency
* [YAML support for matlab](https://github.com/ewiger/yamlmatlab)
* [force_control](https://github.com/yifan-hou/force_control)

## Build
This is a ROS package.

If you only need the matlab implementation, no install is necessary. `algorithm/matlab/solvehfvc.m` is self-contained.

## Contents
* */algorithm* Implementation of the hybrid servoing algorithm, in both matlab and c++.
* */examples/* (under development) Examples of solving contact-rich manipulation problems in Matlab.
* */experiments/* (under development) Run experiments of the examples. Use Matlab GUI with c++ solver.
