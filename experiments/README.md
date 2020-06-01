(This folder is still under heavy development)

Run robot experiments.

Each example problem involves the following parts:
1. `experiments/example_name/config`: Parameters of the example as a yaml file.
2. `experiments/example_name/matlab_gui`: a matlab figure GUI for easy control of the experiment.
3. `experiments/example_name/cpp_robot_server`: a c++ ros node that communicate with the GUI by ROS services. It calls the c++ version of the hybrid servoing algorithm.

# Dependency


# How to use
## 1. Generate Jacobian using Matlab Coder
If the example has an entry under the `example/` folder, it means we use matlab symbolic derivation to compute its jacobian. To use the jacobian function in c++, we need to use Matlab Coder to compile it into a c++ library.

Use the following settings in Matlab Coder:
1. Double precision
2. Use `example_name_control.m` under the corresponding folder under `example/` to auto detect input types.
3. When you use the generated code in your C++ project, if you have ` fatal error: tmwtypes.h: No such file or directory`, copy `matlabroot\extern\include\tmwtypes.h` to your project folder.

