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

## Launch cartographer SLAM configured for the robot
function husky_launch_slam() {
	_husky_check_setup
	roslaunch husky_manager cartographer_husky.launch
}

## Launch MoveBase module to worka along Cartographer SLAM and the rest of the robot modules
function husky_launch_nav() {
	_husky_check_setup
	roslaunch husky_manager nav.launch
}

## Launch rviz session with some usefull topics preconfigured to be shown
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

## Finish cartographer trajectory so a serialization can be performed, then stores the pbstream in a given location
function husky_serialize_cartographer_output()
{
	# Finish the first trajectory. No further data will be accepted on it.
	rosservice call /finish_trajectory 0

	# Ask Cartographer to serialize its current state.
	# (press tab to quickly expand the parameter syntax)
	FOLDER_DATE=$(date +%G_%m_%d_%H_%M)
	MAP_PATH=~/map_log/$FOLDER_DATE
	mkdir -p $MAP_PATH
	rosservice call /write_state "{filename: '${MAP_PATH}/map.pbstream', include_unfinished_submaps: "true"}"
	( cd ${MAP_PATH} && cartographer_pbstream_to_ros_map -pbstream_filename  ${MAP_PATH}/map.pbstream )
	echo -e "${GREEN}Stored serialized data in: ${MAP_PATH}/map.pbstream${NC}"
}

## Command to compile husky workspace from any path
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
	- ${GREEN}husky_record_rosbag${NC} -> records a complete rosbag in ~/test_log with a folder name based on the timetag of the execution time.
	- ${GREEN}husky_serialize_cartographer_output${NC} -> Serializes the output of Cartographer finishing its current trajectory and stores pbstream file with its current state.
	"
}