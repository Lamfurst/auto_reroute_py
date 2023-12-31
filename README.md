# Autoware-CARLA-simulator Waypoint Loop Package

This package provides a simple implementation to send waypoints to the Autoware-CARLA simulator in order to achieve loop running in a specific map.

![Class Diagram](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/Lamfurst/auto_reroute_py/main/demo/flowchart.puml)


https://github.com/Lamfurst/auto_reroute_py/assets/66519297/37beca93-64b0-4056-a517-9b69166341dd


## Prerequisites

Before using this package, ensure that you have the following prerequisites installed:

- ROS2 (Robot Operating System)
- CARLA simulator
- AutoWare
- Follow Dr. Hatem's instructions to install the [Autoware-CARLA bridge and open planner](https://www.youtube.com/watch?v=EFH-vVxn180)
- tmux

## Installation

1. Follow the [doc](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to create a ROS2 workspace
2. Clone this repository into your src folder under your workspace
    ```shell
    cd <your_workspace>/src
    ```
3. Build your workspace using `colcon build`

## Configuration
1. Add or delete the waypoints in waypoints.yaml file under config folder.
2. Change the waypoints_file arg to your own waypoints file path in autoreroute.launch.xml file under launch folder
3. Change the paths in launch_script.sh to your own paths of Carla, autoware folder, current workspace

## Usage
1. Launch Carla and Autoware
2. Launch the waipoint loop by running launch_script.sh
    ```shell
    ./launch_script.sh
    ```
3. If you want to spawn random traffic in the carla, run the following command
    ```shell
    ./launch_script.sh true
    ```
