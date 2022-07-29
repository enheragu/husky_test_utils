#!/usr/bin/env bash

source $HUSKY_SETUP_SCRIPT_PATH/log_utils.sh

## Get IP in eno1 adapter (cisco router)
function _husky_get_ip() {
	export OWN_IP=$(ifconfig eno1 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
	if [ -z "$OWN_IP"  ]
	then
		export OWN_IP=127.0.0.1
	fi
}

## Get ttyUSB port in which IMU is connected
function _husky_get_imu_port() {
	export IMU_PORT="/dev/$(dmesg | grep "tty" | sed -En 's/.*usb 1-4: FTDI USB Serial Device converter now attached to (port:)?((ttyUSB[0-9])).*/\2/p')"
	
	if [ -z "$IMU_PORT"  ]
	then
		export IMU_PORT="/dev/ttyUSB0"
	fi
}

## IP and port configuration
function _husky_export_ip() {
	_husky_get_ip
	_husky_get_imu_port
	export HUSKY_OBC_IP=$OWN_IP 					# Husky on board computer IP 
	export HUSKY_LIDAR_IP=169.254.252.240 			# LIDAR IP
	export HUSKY_LIDAR_IP_DEST=169.254.123.145 	# IP to which LIDAR sends UDP data
	export HUSKY_GPS_PORT="/dev/ttyS0"			# GPS serial port
	export HUSKY_IMU_PORT=$IMU_PORT				# IMU serial port
	
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
