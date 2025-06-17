#!/bin/bash

# Set the name for your tmux session
SESSION_NAME="deployment"

# Start a new detached tmux session
tmux start-server
tmux new-session -d -s $SESSION_NAME

# Send the first command to the initial pane (pane 0)
tmux send-keys -t $SESSION_NAME:0 "cd .devcontainer && docker compose -f compose.multi-arch.yml run --rm deploy_vlm" C-m

sleep 0.5
# Split the window vertically to create a second pane
tmux split-window -h

# Send the second command to the newly created pane (pane 1)
tmux send-keys -t $SESSION_NAME:1 "cd .devcontainer && docker compose -f compose.multi-arch.yml run --rm deploy_vlm" C-m

# Attach to the tmux session to view the panes
tmux attach-session -t $SESSION_NAME