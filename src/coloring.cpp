/*
    Coloring PointCloud (SQ_LiDAR and ZED)

    author : Yudai Sadakuni
*/

#include <ros/ros.h>
#include "ros/package.h"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;

PointCloud2 pc_msg;

void pc_callback(const PointCloud2ConstPtr msg)
{
    pc_msg = *msg;
}


void callback(const ImageConstPtr& image_msg,
              const CameraInfoConstPtr& cinfo_msg)
{
    cout<<"ALL GREEN"<<endl;
    
    // cv_bridge
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;
    
    // realsenseはBGRでデータが格納されているため, opencv用にRGB値に変換する
    cv::Mat image_rgb;
    cv::cvtColor(image ,image_rgb, CV_BGR2RGB);
    
    // camera info
    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    // Coloring Step
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr area(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pc_msg, *cloud);
	cloud->header.frame_id = pc_msg.header.frame_id;
	
	cout<<"Input Size : "<<cloud->points.size()
		<<" Frame : "<<pc_msg.header.frame_id<<endl;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud->points.begin(); pt < cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image_rgb.cols && uv.y > 0 && uv.y < image_rgb.rows)
        {
            pcl::PointXYZRGB p;
            p.x = (*pt).x;
            p.y = (*pt).y;
            p.z = (*pt).z;
            p.b = image_rgb.at<cv::Vec3b>(uv)[0];
            p.g = image_rgb.at<cv::Vec3b>(uv)[1];
            p.r = image_rgb.at<cv::Vec3b>(uv)[2];

            area->points.push_back(p);
        }
    }
    
    cout<<"Points size : "<< area->points.size() << endl;

    PointCloud2 output;
    pcl::toROSMsg(*area, output);
    output.header.frame_id = pc_msg.header.frame_id;
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coloring");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("/cloud", 10, pc_callback);

	message_filters::Subscriber<Image> image_sub(nh, "/image", 10);
	message_filters::Subscriber<CameraInfo> cinfo_sub(nh, "/camera_info", 10);
	
	typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cinfo_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 10);

    ros::spin();

    return 0;
}
