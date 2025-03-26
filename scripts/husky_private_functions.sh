#!/usr/bin/env bash

source $HUSKY_SETUP_SCRIPT_PATH/log_utils.sh

## Get IP in eno1 adapter (cisco router)
function _husky_get_ip() {

	## Setup base (wheel...) serial port
	export HUSKY_PORT="/dev/$(dmesg | grep "tty" | sed -En 's/.*usb 1-[0-9]: pl2303 converter now attached to (port:)?((ttyUSB[0-9])).*/\2/p')"
	
	if [ -z "$HUSKY_PORT"  ]
	then
		export HUSKY_PORT="/dev/ttyUSB0"
	fi

	## Setup OBC own IP
	export OWN_IP=$(ifconfig eno1 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
	if [ -z "$OWN_IP"  ]
	then
		export OWN_IP=127.0.0.1
	fi
}

## Get ttyUSB port in which IMU is connected
function _husky_get_imu_port() {
	export IMU_PORT="/dev/$(dmesg | grep "tty" | sed -En 's/.*usb 1-[0-9]: FTDI USB Serial Device converter now attached to (port:)?((ttyUSB[0-9])).*/\2/p')"
	
	if [ -z "$IMU_PORT"  ]
	then
		export IMU_PORT="/dev/ttyUSB0"
  	else
   		export IMU_PORT="$IMU_PORT"
	fi
}

## IP and port configuration
function _husky_export_ip() {
	_husky_get_ip
	_husky_get_imu_port
	_husky_setup_urdf
	export HUSKY_OBC_IP=$OWN_IP 					# Husky on board computer IP 
	export HUSKY_LIDAR_IP=$(getent hosts os-122138000706.local | awk '{print $1}') 			# LIDAR IP ORIGINAL
	export HUSKY_LIDAR_IP_DEST=$(ip addr show enp2s0 | grep "inet " | awk '{print $2}' | cut -d/ -f1)
	# export HUSKY_LIDAR_IP_DEST=169.254.97.224 
	export HUSKY_GPS_PORT="/dev/ttyS0"			# GPS serial port
	export HUSKY_IMU_PORT=$IMU_PORT				# IMU serial port
	
	export FRONTAL_FISHEYE_IP="169.254.165.4"
    export REAR_FISHEYE_IP="169.254.165.3"
	export MULTIESPECTRAL_VISIBLE_IP="169.254.165.5"
	export MULTIESPECTRAL_LWIR_IP="169.254.165.138"

	export HUSKY_WIFI_IP=$(ip addr show wlp3s0 | grep "inet " | awk '{print $2}' | cut -d/ -f1)

	export ROS_MASTER_URI=http://$OWN_IP:11311	# ROS Master IP
	export ROS_IP=$OWN_IP						# ROS IP of this OBC
}


## Check if setup was already sourced and env vars are loaded. If not, load them.
function _husky_check_setup() {
	if [[ $ROS_CONFIGURED -eq 0 ]]
	then
		husky_ros_setup
	else
		print_info "ROS ws was already configured, not resourcing it."
	fi
}

## Sychronize time of the internal clock of the Ouster with the OBC time
function _husky_lidar_sync_time() {
	CHECK_FILE="/tmp/.husky_lidar_synced_true.check"
	if [[ -f $CHECK_FILE ]]
	then
		print_green "LIDAR was already synced."
	else
		print_green "Synchronizing LIDAR clock with OBC current timestamp."
		_husky_check_setup
		
		sudo systemctl daemon-reload
		sudo systemctl restart ptp4l
		touch $CHECK_FILE
	fi
}

## Configure extras for URDF of base husky
function _husky_setup_urdf() {
	## Disable default EKF as it will be loaded later including Ouster IMU
	export ENABLE_EKF=false

	## Configure file with extras to be included in URDF
	export HUSKY_URDF_EXTRAS="$HUSKY_SETUP_SCRIPT_PATH/../urdf/husky_extras.urdf.xacro"
	
	## Export IMU setup
	um7_imu_enclosure_x_size=0.028
	um7_imu_enclosure_y_size=0.028
	um7_imu_enclosure_z_size=0.009
	
	# export HUSKY_IMU_XYZ="$(echo "0.291+$um7_imu_enclosure_x_size*0.5" | bc) 0 $(echo "0.284-$um7_imu_enclosure_z_size*0.5" | bc)" 
	export HUSKY_IMU_XYZ=" -0.025 0.0 $(echo "0.202+$um7_imu_enclosure_z_size*0.5"| bc)"
	export HUSKY_IMU_RPY="0 0 0"
	export HUSKY_IMU_PARENT="top_plate_link"
		
	#export HUSKY_LIDAR_XYZ="$(echo "0.291+$um7_imu_enclosure_x_size*0.5" | bc) 0 0.361925"
	export HUSKY_LIDAR_XYZ="-0.055 0 0.585" 
	export HUSKY_LIDAR_RPY="0 0 0"
	
}


# function _husky_flir_setup()
# {
# 	# CHECK_FILE="/tmp/.husky_flir_configured_true.check"
# 	# if [[ -f $CHECK_FILE ]]
# 	# then
# 	# 	print_green "FLIR camera was already synced."
# 	# else
# 	# 	echo "Clearing previous XML cahce  data from A68 camera"
# 	# 	rm ~/.config/spinnaker/xml/A68_M0000726_GenICam.xml
# 	# 	print_green "Configuring FLIR camera to have correct IP."
# 	# 	# Strange IP appears in paralel in enp1s0 to the Static one configured...
# 	# 	# Good bye to it :)
# 	# 	# (cd  /opt/spinnaker/bin && ./GigEConfig -a)
# 	# 	# (cd  /opt/spinnaker/bin && ./GigEConfig -s M0000726 -i 192.168.4.6 -n 255.255.255.0 -g 192.168.4.2)
# 	# 	touch $CHECK_FILE
# 	# fi
# }


function _multiespectral_capture_init()
{   
    TOPIC_NAME="/Multiespectral/AS/goal"
    TIMEOUT=15
    ELAPSED=0

    while ! rostopic list | grep -q "$TOPIC_NAME"; do
        if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
            echo "***** Timeout reached. $TOPIC_NAME server not found. Exiting. *****"
            return 1
        fi

        echo "***** Waiting for $TOPIC_NAME server... *****"
        sleep 1
        ELAPSED=$((ELAPSED + 1))
    done

    echo "***** Activate goal for $TOPIC_NAME. *****"
    
    rostopic pub /Multiespectral/AS/goal multiespectral_fb/MultiespectralAcquisitionActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  store: false"
}


function _husky_launch_multiespectral_camera() 
{
	# _husky_flir_setup
	# Launch goal to AC to capture images by default from the beginning
	_multiespectral_capture_init &	
	roslaunch multiespectral_fb multiespectral.launch
}