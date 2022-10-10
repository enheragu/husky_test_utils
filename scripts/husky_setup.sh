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

_husky_setup_urdf

## Prepares setup with ROS setup script and configuring IPs and ports to be used in robot
function husky_ros_setup() {
	_husky_export_ip
	source ~/cartographer/devel_isolated/setup.sh
	source ~/husky_noetic_ws/devel/setup.bash

	export ROS_CONFIGURED=1
	
	print_green "Husky ROS development environment loaded"
}

## Launch sensosrs interfaces (GPS, LIDAR and IMU)
function husky_launch_sensors() {
	_husky_lidar_sync_time
	_husky_check_setup
	roslaunch husky_manager sensors_manager.launch $1
}

## Launch base setup (base, control and teleoperation)
# It is launched by default when initin computer, check if you need
# to manually relaunch it
function husky_launch_base() {
	_husky_check_setup
	#roslaunch husky_base base.launch 
	sudo systemctl restart ros
}

function husky_launch_slam() {
	_husky_check_setup
	roslaunch husky_manager cartographer_husky.launch
}

function husky_launch_rviz() {
	_husky_check_setup
	rosrun rviz rviz -d $HUSKY_SETUP_SCRIPT_PATH/../rviz/rviz_map_cfg.rviz
}

## Records rosbag with all topics. It is stored into ~/test_log/ folder under a folder
# with current date as name
function husky_record_rosbag() {
	_husky_check_setup
	FOLDER_DATE=$(date +%G_%m_%d)
	ROSBAG_PATH=~/test_log/$FOLDER_DATE
	mkdir -p $ROSBAG_PATH
	cd $ROSBAG_PATH
	rosbag record -a
}

function husky_make() {
	_husky_check_setup
	(cd ~/husky_noetic_ws/ &&  catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release)
}

function husky_launch_help() {
	echo -e "The following commands are available to configure launch different packages of the robot:
	- ${GREEN}husky_ros_setup${NC} -> Command to setup workspace and enviromental variables to work with the different sensors.
	- ${GREEN}husky_launch_base${NC} -> Launch base package of the robot enabling control, teleoperation and interface with the platform.
	- ${GREEN}husky_launch_sensors${NC} -> Lanuch the sensors package that enable IMU, LIDAR and GPS messages to be produced. Also launches localization EKF to fuse odometry with both IMU information (UM7 and the one integrated with ouster LIDAR).
	- ${GREEN}husky_launch_slam${NC} -> Lanuch cartographer node to produce SLAM.
	- ${GREEN}husky_launch_rviz${NC} -> Launches an rviz interface prsetted with some useful information.
	
Other commands that might help are:
	- ${GREEN}husky_make${NC} -> allows to make the husky customization workspace from any path.
	- ${GREEN}husky_record_rosbag${NC} -> records a complete rosbag in ~/test_log with a folder name based on the timetag of the execution time
	 "
}