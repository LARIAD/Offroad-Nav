#!/bin/bash

source /workspace/barakuda_isaac_sim/mono_depth_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:13312
rosparam set /use_sim_time true
export PYTHONPATH="${PYTHONPATH}:/workspace/barakuda_isaac_sim/mono_depth_ws/src/depth_rescaling/src/depth_estimation/depth_models/depth_anything/Depth-Anything-V2/"
roslaunch depth_estimation nav_depth_estimation.launch