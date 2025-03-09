#!/bin/bash

# Local and remote folder details
LOCAL_FOLDER="$HOME/ros2_ws/src/spiral_zipper_turret/"
PI_USER="turret"
PI_HOST="192.168.20.120"
REMOTE_FOLDER="~/ros2_ws/src/spiral_zipper_turret/"

# Rsync options
# -a : Archive mode (preserves permissions, timestamps, etc.)
# -v : Verbose output
# --delete : Deletes files on the remote folder that are not in the local folder
# --exclude : Excludes patterns (e.g., hidden files and .git)
RSYNC_OPTIONS="-av --delete --exclude='.*' --exclude='.git/'"

# Perform the sync
echo "Starting sync from $LOCAL_FOLDER to $PI_USER@$PI_HOST:$REMOTE_FOLDER"
rsync $RSYNC_OPTIONS "$LOCAL_FOLDER" "$PI_USER@$PI_HOST:$REMOTE_FOLDER"

# Check if the sync was successful
if [ $? -eq 0 ]; then
    echo "Sync completed successfully!"
else
    echo "Sync failed. Please check your connection or folder paths."
    exit 1
fi
