/*
 * calibration accuracy evaluation
 * 
 * author : Yudai Sadakuni
 *
 */

#include <calibration/function.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <calibration/calibration_lidarConfig.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

typedef pcl::PointXYZRGB PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

// このパラメータで取得する点群のエリアを確定する
// キャリブレーションボードが収まる範囲に調整すること
// rqt_reconfigureで変更可能
double MIN_Z, MAX_Z;
double MIN_DIS, MAX_DIS;
double MIN_ANG, MAX_ANG;

// segmentation用
double SEGMENT_DISTANCE;

ros::Publisher pub_pickup;
ros::Publisher pub_outlier;
ros::Publisher pub_plane;
ros::Publisher pub_histgram;

ros::Publisher pub_plane_transform;
ros::Publisher pub_area_filter;

// 中間値法
// RGBの中間値を利用してグレイスケール化する
template<class T_Ptr>
void middle_value(T_Ptr rgb_cloud,
                  int histgram[])
{
    std::cout<<"point size:"<<rgb_cloud->points.size()<<std::endl;

    for(size_t i=0;i<rgb_cloud->points.size();i++)
    {
        int list[3] = {};
        list[0] = rgb_cloud->points[i].r;
        list[1] = rgb_cloud->points[i].g;
        list[2] = rgb_cloud->points[i].b;

        int min = list[0];
        int max = list[0];
        for(int i=1;i<3;i++)
        {
            if(list[i]<min)
                min = list[i];
            if(max<list[i])
                max = list[i];
        }
        int Y = (max+min)/2; 
        histgram[Y] += 1;
    }   
}

// NTSC係数による加重平均法
template<class T_Ptr>
void ntsc_coef(T_Ptr rgb_cloud,
               int histgram[])
{
    std::cout<<"point size:"<<rgb_cloud->points.size()<<std::endl;

    for(size_t i=0;i<rgb_cloud->points.size();i++)
    {
        int R = rgb_cloud->points[i].r;
        int G = rgb_cloud->points[i].g;
        int B = rgb_cloud->points[i].b;
        int Y = ( 0.298912 * R + 0.586611 * G + 0.114478 * B );
        histgram[Y] += 1;
    }
}

// HDTV係数による加重平均と補正
template<class T_Ptr>
void hdtv_coef(T_Ptr rgb_cloud,
               int histgram[])
{
    std::cout<<"point size:"<<rgb_cloud->points.size()<<std::endl;
    
    int X = 2.2;

    for(size_t i=0;i<rgb_cloud->points.size();i++)
    {
        int R = pow(rgb_cloud->points[i].r, X)*0.222015;
        int G = pow(rgb_cloud->points[i].g, X)*0.706655;
        int B = pow(rgb_cloud->points[i].b, X)*0.071330;

        int Y = pow(R+G+B, 1/X);
        histgram[Y] += 1;
    }
}


void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr input (new CloudA);
    pcl::fromROSMsg(*msg, *input);

    // Pick Up Cloud
    CloudAPtr area(new CloudA);
    pickup_cloud<CloudAPtr>(input,
                            area,
                            MIN_Z,
                            MAX_Z,
                            MIN_ANG,
                            MAX_ANG,
                            MIN_DIS,
                            MAX_DIS);
    
    // outlier removal
    if (!area->points.size())
        return;
    CloudAPtr outlier(new CloudA);
    outlier_removal<PointA, CloudAPtr>(area, outlier, 100, 1.0);

    // Plane Segmentation
    if(!outlier->points.size())
        return;
    CloudAPtr plane(new CloudA);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    plane_segmentation<PointA, CloudA, CloudAPtr>(outlier, plane, coefficients, inliers, SEGMENT_DISTANCE);
    std::cout<<"coefficients "<<
        coefficients->values[0]<<" "<<
        coefficients->values[1]<<" "<<
        coefficients->values[2]<<" "<<
        coefficients->values[3]<<std::endl;
    
    // plane centroid
    Eigen::Vector3f plane_centroid;
    calc_centroid<CloudAPtr>(plane, plane_centroid);
    std::cout<<"plane centroid "<<plane_centroid.transpose()<<std::endl;

    // transform
    string laser_frame=msg->header.frame_id;
    string board_frame="calibration_board_lidar";
    tf::Transform tf;
    transform(plane_centroid, coefficients, tf, laser_frame, board_frame);

    // transform pointcloud
    CloudAPtr trans_cloud(new CloudA);
    transform_pointcloud(plane, trans_cloud, tf.inverse());

    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_centroid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ circle1, circle2, circle3, circle4;
    circle1.x=0; circle1.y=0.15;  circle1.z=0.15; 
    circle2.x=0; circle2.y=0.15;  circle2.z=-0.15;
    circle3.x=0; circle3.y=-0.15; circle3.z=0.15;
    circle4.x=0; circle4.y=-0.15; circle4.z=-0.15;
    circle_centroid->points.push_back(circle1);
    circle_centroid->points.push_back(circle2);
    circle_centroid->points.push_back(circle3);
    circle_centroid->points.push_back(circle4);

    for(size_t i=0;i<circle_centroid->points.size();i++)
        std::cout<<"x:"<<circle_centroid->points[i].x 
                 <<" y:"<<circle_centroid->points[i].y
                 <<" z:"<<circle_centroid->points[i].z<<std::endl;

    // circle area outlier
    CloudAPtr area_filter(new CloudA);
    for(size_t i=0;i<trans_cloud->points.size();i++){
        for(size_t j=0;j<circle_centroid->points.size();j++){
            double distance = sqrt( pow(trans_cloud->points[i].x, 2) + 
                                    pow(trans_cloud->points[i].y-0.15, 2) + 
                                    pow(trans_cloud->points[i].z-0.15, 2) );
            if(0.10<distance){
                area_filter->points.push_back(trans_cloud->points[i]);
                break;
            }
        }
    }

    // histgram
    int histgram[255] = {};
    middle_value(plane, histgram);

    std_msgs::Int32MultiArray data_array;
    for(int i=0;i<255;i++)
        data_array.data.push_back(histgram[i]);
    pub_histgram.publish(data_array);

    // histgramを元にしきい値を設定してください
    // rosrun calibration histgram.py
    int threshold = 100;
    int count = 0;

    for(int i=0;i<threshold;i++)
        count += histgram[i];

    int size = int(plane->points.size());

    printf("error:%.3f(%) size:%d/%d\n", double(count)/size*100, count, int(plane->points.size()));

    pub_cloud(area, msg->header, pub_pickup);
    pub_cloud(outlier,  msg->header, pub_outlier);
    pub_cloud(plane, msg->header, pub_plane);
    pub_cloud(trans_cloud, msg->header, pub_plane_transform);
    pub_cloud(area_filter, msg->header, pub_area_filter);
}

bool first_flag = true;
void callback(calibration::calibration_lidarConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f", 
            config.min_z,   config.max_z,
            config.min_ang, config.max_ang,
            config.min_dis, config.max_dis,
            config.seg_dis);
    if(first_flag)
        first_flag = false;
    else{
        MIN_Z = config.min_z;
        MAX_Z = config.max_z;
        MIN_ANG = config.min_ang;
        MAX_ANG = config.max_ang;
        MIN_DIS = config.min_dis;
        MAX_DIS = config.max_dis;
        SEGMENT_DISTANCE = config.seg_dis;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_lidar");
    ros::NodeHandle nh("~");

    nh.param<double>("min_z",   MIN_Z,   -0.5);
    nh.param<double>("max_z",   MAX_Z,   0.5);
    nh.param<double>("min_ang", MIN_ANG, -0.95);
    nh.param<double>("max_ang", MAX_ANG, 0.87);
    nh.param<double>("min_dis", MIN_DIS, 1.0);
    nh.param<double>("max_dis", MAX_DIS, 1.5);
    nh.param<double>("seg_dis", SEGMENT_DISTANCE, 0.05);

    // Subscriber
    ros::Subscriber sub = nh.subscribe("/cloud", 10, pcCallback);
    // Publisher
    pub_histgram = nh.advertise<std_msgs::Int32MultiArray>("/histgram", 10);
    pub_pickup   = nh.advertise<sensor_msgs::PointCloud2>("/cloud/pickup", 10);
    pub_outlier  = nh.advertise<sensor_msgs::PointCloud2>("/cloud/outlier", 10);
    pub_plane       = nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 10);
    
    pub_plane_transform = nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane/transform", 10);
    pub_area_filter = nh.advertise<sensor_msgs::PointCloud2>("/cloud/area/filter", 10);

    // dynamic reconfigure
    dynamic_reconfigure::Server<calibration::calibration_lidarConfig> server;
    dynamic_reconfigure::Server<calibration::calibration_lidarConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
