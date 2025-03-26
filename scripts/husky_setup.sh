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

## Useful topic list
export HUSKY_CONTROLLER_TOPICS="/husky_velocity_controller/cmd_vel /husky_velocity_controller/odom /tf /tf_static"
export HUSKY_SENSORS_TOPICS="/gnss/fix /gnss/odom /imu/data /ouster/imu /ouster/points"
export HUSKY_LOCALZATION_TOPICS="/ekf/odom_filtered /ekf2/odom_filtered /navsat/gps_filtered /navsat/odom_gps"

source $HUSKY_SETUP_SCRIPT_PATH/log_utils.sh				# Log utilities to be used
source $HUSKY_SETUP_SCRIPT_PATH/husky_private_functions.sh	# Private helper functions

_husky_setup_urdf

function bind_fkeys() { # Test output with showkey -a command
if [[ $- == *i* ]]; then

	bind -x '"\e[15~":"husky_launch_base"' 					# F5
	bind -x '"\e[17~":"husky_launch_sensors"' 				# F6
	bind -x '"\e[18~":"husky_launch_multiespectral_camera"'	# F7
	bind -x '"\e[19~":"husky_launch_fisheye_cameras"'	    # F8
	bind -x '"\e[20~":"husky_check_sensors"' 				# F9		
else
    echo "Non interactive shell, not binding keys for command shortcuts"
fi
}

## Prepares setup with ROS setup script and configuring IPs and ports to be used in robot
function husky_ros_setup() {
	bind_fkeys
	_husky_export_ip
	source /home/administrator/cartographer/devel_isolated/setup.sh
	source /home/administrator/husky_noetic_ws/devel/setup.bash

	export ROS_HOSTNAME=localhost
	export ROS_MASTER_URI=http://localhost:11311

	# Check if disk is already mounted 
	if ! grep -qs '/media/administrator/data ' /proc/mounts; then
		udisksctl mount -b /dev/disk/by-label/data
		print_green "Mounted disk data"
	fi

	export ROS_CONFIGURED=1

	print_green "Husky ROS development environment loaded"
}

## Launch base setup (base, control and teleoperation)
# It is launched by default when initin computer, check if you need
# to manually relaunch it
function husky_launch_base() {
	_husky_check_setup
	#roslaunch husky_base base.launch
	# sudo systemctl restart ros
	sudo systemctl restart husky_base.service
	systemctl --user restart conky.service
}

## Launch sensosrs interfaces (GPS, LIDAR and IMU)
function husky_launch_sensors() {
	_husky_lidar_sync_time
	_husky_check_setup
	# roslaunch husky_manager sensors_manager.launch $1
	sudo systemctl restart sensors.service
}

# Launch both Basler and LWIR camera pair, with a start goal already
# posted
function husky_launch_multiespectral_camera() 
{
	_husky_check_setup
	# _husky_flir_setup
	# Launch goal to AC to capture images by default from the beginning
	_multiespectral_capture_init &	
	roslaunch multiespectral_fb multiespectral.launch
	# sudo systemctl restart multiespectral_cameras.service
}

# Launch both Fisheye cameras
function husky_launch_fisheye_cameras() 
{
	_husky_check_setup
	# sudo systemctl restart fisheye_cameras.service
	roslaunch husky_manager fisheye_cameras.launch
}

## Command to compile husky workspace from any path
function husky_make() 
{
	_husky_check_setup
	(cd ~/husky_noetic_ws/ &&  catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release)
}

function husky_check_sensors()
{
	_husky_check_setup
	roslaunch husky_manager check.launch
}


## Launch localization integration
function husky_launch_localization() {
	_husky_lidar_sync_time
	_husky_check_setup
	roslaunch husky_manager localization_manager.launch
}



#########################
##    DEPRECATED :)    ##
#########################

# ## Launch MoveBase module to worka along Cartographer SLAM and the rest of the robot modules
# function husky_launch_nav() {
# 	_husky_check_setup
# 	roslaunch husky_manager nav.launch
# }

## Launch cartographer SLAM configured for the robot
# # function husky_launch_slam() {
# 	_husky_check_setup
# 	roslaunch husky_manager cartographer_husky.launch
# }

# ## Launch rviz session with some usefull topics preconfigured to be shown
# function husky_launch_rviz() {
# 	_husky_check_setup
# 	rosrun rviz rviz -d $HUSKY_SETUP_SCRIPT_PATH/../rviz/rviz_map_cfg.rviz
# }

# ## Records rosbag with all topics. It is stored into ~/test_log/ folder under a folder
# # with current date as name
# function husky_record_rosbag() {
# 	_husky_check_setup
# 	FOLDER_DATE=$(date +%G_%m_%d)
# 	ROSBAG_PATH=media/administrator/data/rosbags/$FOLDER_DATE
# 	mkdir -p $ROSBAG_PATH
# 	cd $ROSBAG_PATH
# 	rosbag record -b 0 --chunksize=10240 "$@"
# }

# ## Finish cartographer trajectory so a serialization can be performed, then stores the pbstream in a given location
# function husky_serialize_cartographer_output()
# {
# 	# Finish the first trajectory. No further data will be accepted on it.
# 	rosservice call /finish_trajectory 0

# 	# Ask Cartographer to serialize its current state.
# 	# (press tab to quickly expand the parameter syntax)
# 	FOLDER_DATE=$(date +%G_%m_%d_%H_%M)
# 	MAP_PATH=~/map_log/$FOLDER_DATE
# 	mkdir -p $MAP_PATH
# 	rosservice call /write_state "{filename: '${MAP_PATH}/map.pbstream', include_unfinished_submaps: "true"}"
# 	( cd ${MAP_PATH} && cartographer_pbstream_to_ros_map -pbstream_filename  ${MAP_PATH}/map.pbstream )
# 	echo -e "${GREEN}Stored serialized data in: ${MAP_PATH}/map.pbstream${NC}"
# }


# function husky_launch_help() {
# 	echo -e "The following commands are available to configure launch different packages of the robot:
# 	- ${GREEN}husky_ros_setup${NC} -> Command to setup workspace and enviromental variables to work with the different sensors.
# 	- ${GREEN}husky_launch_base${NC} -> Launch base package of the robot enabling control, teleoperation and interface with the platform.
# 	- ${GREEN}husky_launch_sensors${NC} -> Lanuch the sensors package that enable IMU, LIDAR and GPS messages to be produced. Also launches localization EKF to fuse odometry with both IMU information (UM7 and the one integrated with ouster LIDAR).
# 	- ${GREEN}husky_launch_slam${NC} -> Lanuch cartographer node to produce SLAM.
# 	- ${GREEN}husky_launch_rviz${NC} -> Launches an rviz interface prsetted with some useful information.

# Other commands that might help are:
# 	- ${GREEN}husky_make${NC} -> allows to make the husky customization workspace from any path.
# 	- ${GREEN}husky_record_rosbag${NC} -> records a complete rosbag in ~/test_log with a folder name based on the timetag of the execution time.
# 	- ${GREEN}husky_serialize_cartographer_output${NC} -> Serializes the output of Cartographer finishing its current trajectory and stores pbstream file with its current state.
# 	"
# }

