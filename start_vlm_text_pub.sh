#!/bin/bash
# filepath: /root/vlm_ws/start_vlm_text_pub.sh

# Create a new tmux session with 2 panels in one row
tmux new-session -d -s vlm_session

# Split the window vertically (creates left and right panels)
tmux split-window -h

# Configure left panel (panel 0)
tmux send-keys -t vlm_session:0.0 'cd .devcontainer &&  docker compose -f compose.multi-arch.yml run --rm deploy_vlm' Enter
# Wait for the container to start
sleep 2.5
tmux send-keys -t vlm_session:0.0 'source install/setup.bash' Enter
tmux send-keys -t vlm_session:0.0 'ros2 run vita_agent person_follower' Enter

# Configure right panel (panel 1)
tmux send-keys -t vlm_session:0.1 'cd .devcontainer &&  docker compose -f compose.multi-arch.yml run --rm deploy_vlm' Enter
# Wait for the container to start
sleep 2.5
tmux send-keys -t vlm_session:0.1 'source install/setup.bash' Enter
tmux send-keys -t vlm_session:0.1 'ros2 run vita_agent text_publisher' Enter

# Attach to the session
tmux attach-session -t vlm_session