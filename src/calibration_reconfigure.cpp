#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <calibration/calibrationConfig.h>

void callback(calibration::calibrationConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f", 
            config.min_z,   config.max_z,
            config.min_ang, config.max_ang,
            config.min_dis, config.max_dis);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_seg_plane");

  dynamic_reconfigure::Server<calibration::calibrationConfig> server;
  dynamic_reconfigure::Server<calibration::calibrationConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
