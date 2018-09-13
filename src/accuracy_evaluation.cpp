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

class COLORING{
    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher pub;
    public:
        COLORING();

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr cloud);

        typedef message_filters::sync_policies::Approximate<sensor_msgs::Image, sensor_msgs::CameraInfo> realsense_sync_subs;
        message_filters::Subscriber<Image> image_sub;
        message_filters::Subscriber<CameraInfo> camerainfo_sub;
        message_filters::Subscriber<realsense_sync_subs> realsense_sync;
        void realsense_callback(const sensor_msgs::ImageConstPtr image, const sensor_msgs::CameraInfoConstPtr cinfo);
};

COLORING::COLORING()
    ; nh("~"),
      image_sub(nh, "/image", 10),
      camefainfo_sub(nh, "/cinfo", 10)
{
    nh.param<double>("min_z", MIN_Z);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "accuracy_evaluation");

    ros::spin();

    return 0;
}
