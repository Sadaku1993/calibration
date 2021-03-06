#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
CLOUD="/cloud/tf"
ODOM="/odom"
IMU="/imu/data"
REALSENSE="/camera1/color/image_raw/compressed /camera1/color/camera_info /camera1/ds_cloud"

echo $TIME &
echo $CLOUD &
echo $ODOM &
echo $IMU &
echo $REALSENSE &

/opt/ros/kinetic/bin/rosbag record $CLOUD $ODOM $IMU $REALSENSE -O /home/amsl/$TIME.bag
