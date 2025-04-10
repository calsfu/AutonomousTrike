#!/bin/bash

# Create a new tmux session
session_name="vint_locobot_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# Run roscore in the first pane
tmux select-pane -t 0
tmux send-keys "roscore" Enter

# Run CARLA in the second pane
tmux select-pane -t 1
tmux send-keys "cd ~/Desktop/CARLA" Enter
tmux send-keys "./CarlaUE4.sh" Enter

sleep 10 # wait for CARLA to boot up

# Drive a car in the third pane
tmux select-pane -t 2
tmux send-keys "cd ~/Desktop/CARLA/myScripts" Enter
tmux send-keys "conda activate py3.8" Enter
tmux send-keys "python carla_control_node.py" Enter

# Run explore in the fourth pane
tmux select-pane -t 3
tmux send-keys "conda activate vint_deployment" Enter
tmux send-keys "python explore.py $@" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
