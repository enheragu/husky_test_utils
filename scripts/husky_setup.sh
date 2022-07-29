#!/usr/bin/env bash

# Default
export ROS_CONFIGURED=0

## Get IP in eno1 adapter (cisco router)
function husky_get_ip() {
	export OWN_IP=$(ifconfig eno1 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
	if [ -z "$OWN_IP"  ]
	then
		export OWN_IP=127.0.0.1
	fi
}

## Get ttyUSB port in which IMU is connected
function husky_get_imu_port() {
	export IMU_PORT="/dev/$(dmesg | grep "tty" | sed -En 's/.*usb 1-4: FTDI USB Serial Device converter now attached to (port:)?((ttyUSB[0-9])).*/\2/p')"
	
	if [ -z "$IMU_PORT"  ]
	then
		export IMU_PORT="/dev/ttyUSB0"
	fi
}

## IP and port configuration
function husky_export_ip() {
	husky_get_ip
	husky_get_imu_port
	export HUSKY_OBC_IP=$OWN_IP 					# Husky on board computer IP 
	export HUSKY_LIDAR_IP=169.254.252.240 			# LIDAR IP
	export HUSKY_LIDAR_IP_DEST=169.254.123.145 	# IP to which LIDAR sends UDP data
	export HUSKY_GPS_PORT="/dev/ttyS0"			# GPS serial port
	export HUSKY_IMU_PORT=$IMU_PORT				# IMU serial port
	
	export ROS_MASTER_URI=http://$OWN_IP:11311	# ROS Master IP
	export ROS_IP=$OWN_IP						# ROS IP of this OBC
}

## Setup ros workspace in ~/husky_noetic_ws
function husky_ros_setup() {
	husky_export_ip
	source ~/husky_noetic_ws/devel/setup.bash
	export ROS_CONFIGURED=1
}

## Check if setup was already sourced and env vars are loaded. If not, load them.
function husky_check_setup() {
	if [[ $ROS_CONFIGURED -eq 0 ]]
	then
		husky_ros_setup
	else
		echo "ROS ws was already configured, not resourcing it.\n"
	fi
}

## Sychronize time of the internal clock of the Ouster with the OBC time
function husky_lidar_sync_time() {
	if [[ -f  /tmp/lidar_synced_true.check ]]
	then
		echo "LIDAR was already synced.\n"
	else
		echo "Synchronizing LIDAR clock with OBC current timestamp.\n"
		husky_check_setup
		
		touch /tmp/lidar_synced_true.check
		sudo systemctl daemon-reload
		sudo systemctl restart ptp4l
	fi
}

## Launch sensosrs interfaces (GPS, LIDAR and IMU)
function husky_launch_sensors() {
	husky_check_setup
	roslaunch husky_manager sensors.launch $1
}

