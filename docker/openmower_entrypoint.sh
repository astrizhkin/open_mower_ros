#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash

# setup  environment
source /opt/open_mower_ros/version_info.env

export RECORDINGS_PATH=${RECORDINGS_PATH:-$HOME}
echo "RECORDINGS_PATH=$RECORDINGS_PATH"
export PARAMS_PATH=${PARAMS_PATH:-$HOME}
echo "PARAMS_PATH=$PARAMS_PATH"

# debugging get controlled via env var DEBUG
shopt -s nocasematch
case "${DEBUG:-0}" in
    1|true|yes|on|y)
        export ROSOUT_DISABLE_FILE_LOGGING=False
        unset ROSCONSOLE_CONFIG_FILE
    ;;
    *)
        export ROSCONSOLE_CONFIG_FILE=/config/rosconsole.config
        export ROSOUT_DISABLE_FILE_LOGGING=True
    ;;
esac
shopt -u nocasematch || true

# Ensure stdout and stderr are unbuffered to get logging in real time order
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
export PYTHONUNBUFFERED=1

exec -- "$@"
