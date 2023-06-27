# Autoware-CARLA-simulator Waypoint Loop Package

This package provides a simple implementation to send waypoints to the Autoware-CARLA simulator in order to achieve loop running in a specific map.

<div hidden>
@startuml
partition "Auto_Reroute"{
partition "TF Listener" {
  "Check ego_vehicle's distance to goal" --> if "Dist < MIN_DIST ?" then 
    --> [Yes] "Send new pose"
  else
    -up-> [No] "Check ego_vehicle's distance to goal"

  "Send new pose" --> "Check ego_vehicle's distance to goal"
}
}

partition Autoware {
  partition "Planning"{
    "Send new pose" -up-> [/planning/mission_planning/goal]"mission_planner"
  }
}
partition "Auto_Reroute"{
partition "Planning State Callback"{
  "mission_planner" -up-> [/autoware/state] "Check planning state" 
  "Check planning state"   --> if "WAITING_FOR_ENGAGE?" then
    --> [Yes] "Send Engage Request"
    "Send Engage Request" --> "Check planning state" 
  else
    --> [No] "Check planning state" 
}
}

partition Autoware {
  partition "Control"{
    "Send Engage Request" --> [/api/external/set/engage] "vehicle_cmd_gate"
  }
}
@enduml
</div>

<video width="640" height="360" controls>
  <source src="https://www.example.com/video.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

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
3. If you want to spawn traffic in the carla, run the following command
    ```shell
    ./launch_script.sh true
    ```

Generate a uml for my auto_reroute pakcage based on my description here: 1. start a loop for tf listener 2. If the current goal_pose is none then send the first goal pose  3. If the current the goal_pose is not none and if the distance between ego_vehicle's base_link and goal pose is less than min_distance then reads the waypoint from waypoints.yaml file. then send the goal pose. 4. All sending goal  pose process can be divided into a. get 
