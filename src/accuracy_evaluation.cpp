/*
 * calibration accuracy evaluation
 * 
 * author : Yudai Sadakuni
 *
 */

#include<ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "accuracy_evaluation");

    ros::spin();

    return 0;
}
