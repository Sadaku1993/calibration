/*
    icp matching

    author:Yudai Sadakuni

    memo:
        算出した円の重心４つでicpをかけているため、初期位置が違いすぎるとmatchingがかからない
        初期位置パラメータinit_(x, y, z, roll, pitch, yaw)は大体の設置位置から設定してください

*/

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <ctime>
#include "tinyxml.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace sensor_msgs;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

class icp_transform{
    public:
        icp_transform();
    private:
        void lidar_callback (const PointCloud2ConstPtr& cloud);  
        void camera_callback (const PointCloud2ConstPtr& cloud);
        void transform(const ros::TimerEvent&);
        void tf_broadcast(Eigen::Matrix4d matrix, string target_frame, string source_frame);
        void init_transform(string target_frame, string source_frame, CloudAPtr cloud);

        void pub_cloud(CloudAPtr cloud, std_msgs::Header header, ros::Publisher pub);

        ros::NodeHandle nh;

        ros::Subscriber sub_lidar;
        ros::Subscriber sub_camera;

        ros::Publisher pub_dumy_cloud;

        PointCloud2 cloud2_lidar;
        PointCloud2 cloud2_camera;
        
        CloudAPtr pcl_lidar;
        CloudAPtr pcl_camera;

        ros::Timer timer;

        bool tf_flag;
        bool lidar_flag;
        bool camera_flag;

        double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
        string dumy_frame;
};

// コンストラクタ初期化
icp_transform::icp_transform()
    : nh("~"), pcl_lidar(new CloudA), pcl_camera(new CloudA)
{
    sub_lidar  = nh.subscribe<PointCloud2> ("/lidar" , 10, &icp_transform::lidar_callback, this);
    sub_camera = nh.subscribe<PointCloud2> ("/camera", 10, &icp_transform::camera_callback, this);

    pub_dumy_cloud = nh.advertise<PointCloud2>("/dumy_cloud", 10);

    timer = nh.createTimer(ros::Duration(1.0), &icp_transform::transform,this);

    nh.param<bool>("tf_flag", tf_flag, true);

    nh.param<double>("init_x", init_x, 0);
    nh.param<double>("init_y", init_y, 0);
    nh.param<double>("init_z", init_z, 0);
    nh.param<double>("init_roll", init_roll, 0);
    nh.param<double>("init_pitch", init_pitch, 0);
    nh.param<double>("init_yaw", init_yaw, 0);

    lidar_flag = false;
    camera_flag = false;

    dumy_frame = "dumy_frame";
}

void icp_transform::lidar_callback(const PointCloud2ConstPtr& msg)
{
    cloud2_lidar = *msg;
    pcl::fromROSMsg(*msg, *pcl_lidar);
    lidar_flag = true;
}

void icp_transform::camera_callback(const PointCloud2ConstPtr& msg)
{
    cloud2_camera = *msg;
    pcl::fromROSMsg(*msg, *pcl_camera);
    camera_flag = true;
}

//表示関数
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

// Matrix to rpy
void calc_rpy(Eigen::Matrix4d matrix){
	tf::Matrix3x3 mat_l;
    double roll, pitch, yaw;
    mat_l.setValue(matrix(0, 0), matrix(0, 1), matrix(0, 2),
                   matrix(1, 0), matrix(1, 1), matrix(1, 2),
                   matrix(2, 0), matrix(2, 1), matrix(2, 2));
	mat_l.getRPY(roll, pitch, yaw, 1);
    printf("Roll Pitch Yaw :\n");
    printf("RPY = < %6.3f, %6.3f. %6.3f >\n\n", roll, pitch, yaw);
}

void icp_transform::transform(const ros::TimerEvent&){
  if(lidar_flag && camera_flag)
  {
      CloudAPtr dumy_cloud(new CloudA);
      init_transform(cloud2_lidar.header.frame_id, dumy_frame, dumy_cloud);
      dumy_cloud->header.frame_id = dumy_frame;

      std::cout<<"lidar"<<std::endl;
      for(size_t i=0;i<pcl_lidar->points.size();i++)
          std::cout<<pcl_lidar->points[i]<<std::endl;

      std::cout<<"camera"<<std::endl;
      for(size_t i=0;i<pcl_camera->points.size();i++)
          std::cout<<pcl_camera->points[i]<<std::endl;

      std::cout<<"dumy_cloud"<<std::endl;
      for(size_t i=0;i<dumy_cloud->points.size();i++)
          std::cout<<dumy_cloud->points[i]<<std::endl;

      std_msgs::Header header;
      header.frame_id = dumy_frame;
      pub_cloud(dumy_cloud, header, pub_dumy_cloud);


      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
      pcl::IterativeClosestPoint<PointA, PointA> icp;
      icp.setInputTarget(dumy_cloud);
      icp.setInputSource(pcl_camera);
      CloudA Final;
      icp.align(Final);

      if (icp.hasConverged ())
          transformation_matrix = icp.getFinalTransformation ().cast<double>();
      else
      {
          PCL_ERROR ("\nICP has not converged.\n");
          return ;
      }

      tf_broadcast(transformation_matrix, dumy_cloud->header.frame_id, cloud2_camera.header.frame_id);

      /*
      pcl::IterativeClosestPoint<PointA, PointA> icp;
      icp.setInputTarget(pcl_lidar->makeShared());
      icp.setInputSource(pcl_camera->makeShared());
      
      CloudA Final;
      icp.align(Final);

      if (icp.hasConverged ())
          transformation_matrix = icp.getFinalTransformation ().cast<double>();
      else
      {
          PCL_ERROR ("\nICP has not converged.\n");
          return ;
      }

      //変換matrixを表示する
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
      transformation_matrix = icp.getFinalTransformation ().cast<double>();
      // print4x4Matrix (transformation_matrix);
      // calc_rpy(transformation_matrix);

      tf_broadcast(transformation_matrix, cloud2_lidar.header.frame_id, cloud2_camera.header.frame_id);
      */

      lidar_flag = false;
      camera_flag = false;
  }
}

void icp_transform::init_transform(string target_frame,
                                   string source_frame,
                                   CloudAPtr cloud)
{
    tf::Vector3 vector;
    tf::Quaternion q;

    vector.setValue(init_x, init_y, init_z);
    q = tf::createQuaternionFromRPY(init_roll, init_pitch, init_yaw);

    tf::Transform tf;
    tf.setOrigin(vector);
    tf.setRotation(q);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), target_frame, source_frame));

    pcl_ros::transformPointCloud(*pcl_lidar, *cloud, tf.inverse());
}

void icp_transform::tf_broadcast(Eigen::Matrix4d matrix,
                                 string target_frame,
                                 string source_frame){
    tf::Vector3 vector;
    vector.setValue(matrix(0, 3), matrix(1, 3), matrix(2, 3));

    tf::Matrix3x3 mat_l;
    double roll, pitch, yaw;
    mat_l.setValue(matrix(0, 0), matrix(0, 1), matrix(0, 2),
            matrix(1, 0), matrix(1, 1), matrix(1, 2),
            matrix(2, 0), matrix(2, 1), matrix(2, 2));
    mat_l.getRPY(roll, pitch, yaw, 1);
 
    tf::Quaternion q;
    mat_l.getRotation(q);

    tf::Transform tf;
    static tf::TransformBroadcaster br;
    tf.setOrigin(vector);
    tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), target_frame, source_frame));
}

void icp_transform::pub_cloud(CloudAPtr cloud,
                              std_msgs::Header header, 
                              ros::Publisher pub)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = header.frame_id;
    pub.publish(output);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_transform");

    icp_transform icp_transform;

    ros::spin();

    return 0;
}
