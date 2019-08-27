Run robot experiments.

Each example problem involves the following parts:
1. `experiments/example_name/config`: Parameters of the example as a yaml file.
2. `experiments/example_name/matlab_gui`: a matlab figure GUI for easy control of the experiment.
3. `experiments/example_name/cpp_robot_server`: a c++ ros node that communicate with the GUI by ROS services. It calls the c++ version of the hybrid servoing algorithm.

# Dependency

# How to use
1. If the example has an entry under the `example/` folder, it means we use matlab symbolic derivation to compute its jacobian. To use the jacobian function in c++, we need to use Matlab Coder to compile it into a c++ library.

double precision
use `example_name_control.m` to auto detect input types.
If you have ` fatal error: tmwtypes.h: No such file or directory`, copy `matlabroot\extern\include\tmwtypes.h` to your project folder.