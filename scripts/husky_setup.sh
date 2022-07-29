#!/usr/bin/env bash

## Path of current file
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
export HUSKY_SETUP_SCRIPT_PATH=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )


## Default helper variables
export ROS_CONFIGURED=0

source $HUSKY_SETUP_SCRIPT_PATH/log_utils.sh				# Log utilities to be used
source $HUSKY_SETUP_SCRIPT_PATH/husky_private_functions.sh	# Private helper functions

## Prepares setup with ROS setup script and configuring IPs and ports to be used in robot
function husky_ros_setup() {
	_husky_export_ip
	source ~/husky_noetic_ws/devel/setup.bash
	export ROS_CONFIGURED=1
	
	print_green "Husky ROS development environment loaded"
}

## Launch sensosrs interfaces (GPS, LIDAR and IMU)
function husky_launch_sensors() {
	_husky_lidar_sync_time
	_husky_check_setup
	roslaunch husky_manager sensors.launch $1
}

