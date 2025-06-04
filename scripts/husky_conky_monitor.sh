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

echo "[husky_conky_monitor.sh] Loading environment variables..."
source $CURRENT_SCRIPT_PATH/husky_setup.sh
_husky_check_setup

# sudo -u administrator conky -d -c $CURRENT_SCRIPT_PATH/../monitor/conkyrc_network &
# sudo -u administrator conky -d -c $CURRENT_SCRIPT_PATH/../monitor/conkyrc

# Load correct environment variables to run conky
env > /tmp/current_env
run_conky_with_env() {
    sudo -E -u administrator bash -c "
        while IFS='=' read -r key value; do
            export \"\$key=\$value\"
        done < /tmp/current_env
        
        conky -d -c $1
    "
}

run_roslaunch_with_env() {
    sudo -E -u administrator bash -c "
        while IFS='=' read -r key value; do
            export \"\$key=\$value\"
        done < /tmp/current_env
        
        roslaunch husky_manager check.launch node_name:=conky_check_sensors > /tmp/conky_check_sensors.log 2>&1 &
    "
}

echo "[husky_conky_monitor.sh] Running both conky instances"
run_conky_with_env "$CURRENT_SCRIPT_PATH/../monitor/conkyrc_network" &
run_conky_with_env "$CURRENT_SCRIPT_PATH/../monitor/conkyrc" &

echo "[husky_conky_monitor.sh] Launching check sensors node for conky"
run_roslaunch_with_env