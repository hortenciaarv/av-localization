# slm_sim Package

This ROS2 package simulates the movement of a pendulum using RViz. 

## Installation

### Prerequisites

- ROS Humble installed on Ubuntu 22
- RViz
- joint-state-publisher-gui

### Installation Steps

1. Clone this package into your ROS2 workspace:

   ```bash
   cd /path/to/your/ros2/workspace/src
   git clone [https://github.com/soyhorteconh/Team_IntegrationRobotics_and_IntelligentSystems]
2. Build the package using colcon:
```bash
colcon build --symlink-install --packages-select slm_sim
```
3. Source the setup file:
``` bash
source install/setup.bash
```
## Running the Simulation
1. Launch the simulation using the following command:
``` bash
ros2 launch slm_sim display.launch.py
```
2. Open RViz in another terminal:
``` bash
rviz2 -d /path/to/your/workspace/install/slm_sim/share/slm_sim/pendulum.rviz
```
## Demonstration
A video demonstrating the simulation can be found here.
[![Demo del Proyecto](http://img.youtube.com/vi/uIf0BEA_Obo/0.jpg)]( https://youtu.be/uIf0BEA_Obo)

## Additional Note
To install the joint-state-publisher-gui, you can use the following command:
``` bash
sudo apt install ros-humble-joint-state-publisher-gui
```
