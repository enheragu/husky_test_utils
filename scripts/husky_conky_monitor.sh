#!/usr/bin/env bash

## Path of current file
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
CURRENT_SCRIPT_PATH=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )

export DISPLAY=:0

# Wait for X session to be active
SESSION=":0"
while true; do
  if [ -e "/tmp/.X11-unix/X${SESSION#:}" ]; then
    echo "La sesión $SESSION está activa."
    break
  fi
  sleep 1
done

source $CURRENT_SCRIPT_PATH/husky_setup.sh
_husky_check_setup
roscore & 
rosrun husky_manager check_sensors.py __name:=conky_check_sensors >/dev/null 2>&1 &


conky -d -c $CURRENT_SCRIPT_PATH/../monitor/conkyrc