# The-power-of-Multi-agent-systems-in-Search-and-Rescue-Missions

This repository houses the implementation of the Master thesis named above

## Navigating the repository
* `MovevementUnc` contains all code necessary to run the random baseline.
* `MultiQLearning` contains all code necessary to run the Simultaneous category.
* `MultiQsub` contains all code necessary to run the Substitute category.
* `MultiQLearning` contains all code necessary to run the Stress test.


## Running each experiment
The code is based on a combined implementation of gazebo and ROS2, thus to run each experiment this environemt has to be guaranteed.

To run the code these packages need to be installed additionally to ROS2 Jazzy:
* `Open-Cv`

The Crazyflie wall following tutorial has been used as a basis for these implementations.
The tutorial can be found under https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/.


Once the code has been downloaded, situate yourself in the given experiment folder you want to run.
* First source your ros2 codebase `source /opt/ros/{your-ros2-DIST}/setup.bash`
* Next you need to build the codebase `colcon build`
* Source your setup file `source install/local_setup.bash`
* Execute the simulated environment `ros2 launch LaunchSup.py`

