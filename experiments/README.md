Run robot experiments.

Each example problem involves the following parts:
1. `experiments/example_name/config`: Parameters of the example as a yaml file.
2. `experiments/example_name/matlab_gui`: a matlab figure GUI for easy control of the experiment.
3. `experiments/example_name/cpp_robot_server`: a c++ ros node that communicate with the GUI by ROS services. It calls the c++ version of the hybrid servoing algorithm.

# How to add a new experimental setting in track2d
1. Create a new folder under config/ for your setting. Copy the three config files and edit accordingly.
2. Generate your csv trajectory file under data/. Reference it in your config files.
3. Create a launch file that load the new config files.
