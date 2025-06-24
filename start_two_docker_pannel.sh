#!/bin/bash
# filepath: /root/vlm_ws/start_two_docker_pannel.sh

# Create a new tmux session with 4 panels in one window
tmux new-session -d -s docker_session

# Split window into 4 panes (2x2 grid)
tmux split-window -h -t docker_session:0         # Split window horizontally (left/right)
tmux split-window -v -t docker_session:0.0       # Split left pane vertically (top/bottom)
tmux split-window -v -t docker_session:0.1       # Split right pane vertically (top/bottom)

# Start an empty docker shell in each panel
for i in 0 1 2 3; do
    tmux send-keys -t docker_session:0.$i 'make deploy-run-arm64' Enter
done

# Attach to the session
tmux attach-session -t docker_session
