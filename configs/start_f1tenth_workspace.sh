#!/bin/bash
SESSION_NAME="f1tenth_dev"

tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? == 0 ]; then
  echo "Session $SESSION_NAME already exists. Attaching..."
  tmux attach-session -t $SESSION_NAME
  exit 0
fi

tmux new-session -d -s $SESSION_NAME -n "Main"
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux send-keys -t 0 "cd ~/f1tenth_ws" C-m

tmux new-window -n "Monitor"
tmux send-keys -t "Monitor" "cd ~/f1tenth_ws" C-m

tmux select-window -t $SESSION_NAME:Main
tmux attach-session -t $SESSION_NAME
