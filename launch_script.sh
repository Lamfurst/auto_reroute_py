#!/bin/bash
# Parse command-line argument
TRAFFIC=$1

tmux split-window -v
# Check if traffic is true
if [ "$TRAFFIC" == "true" ]; then
    # Run the python script to spawn vehicles
    tmux send-keys "python3 /home/artemis/Workspace/carla-0.9.13/PythonAPI/examples/generate_traffic.py --asynch --safe -n 50 -w 100" C-m
    # Delay for a few seconds
    sleep 5
fi

# launch reroute node in a new terminal
tmux split-window -h
tmux send-keys "source /home/artemis/Workspace/autowarefoundation/carla-autoware-universe/autoware/install/setup.bash" C-m
tmux send-keys "source /home/artemis/personal_autoware_ws/install/setup.bash" C-m
tmux send-keys "ros2 launch launch/autoreroute.launch.xml" C-m