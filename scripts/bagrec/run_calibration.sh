#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# Launch SQ Lidar
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra run_sq2_for_joy.launch' --geometry=50x12+0+0 &
sleep 1s

# Launch realsense and Downsample
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/lidar_camera_calibration/scripts/realsense/downsample_realsense1.sh' --geometry=50x12+0+250 &
sleep 1s

# Launch Sensor TF
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_tf.launch' --geometry=50x12+600+0 &
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion sq2_realsense.launch' --geometry=50x12+600+250 &
sleep 1s

# Transform PointCloud from /centerlaser_ to /centerlaser
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_transform_pointcloud.launch' --geometry=50x12+600+500 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/calibration.rviz' --geometry=50x12+600+750 &
sleep 5s

# bagrec
# gnome-terminal -e '/home/amsl/ros_catkin_ws/src/calibration/scripts/bag_rec_calibration.sh' --geometry=50x12+1000+0 &
