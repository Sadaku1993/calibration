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
        void tf_broadcast(Eigen::Matrix4d matrix);
        void save_launch(Eigen::Matrix4d matrix);

        ros::NodeHandle nh;

        ros::Subscriber sub_lidar;
        ros::Subscriber sub_camera;

        PointCloud2 cloud2_lidar;
        PointCloud2 cloud2_camera;
        
        CloudAPtr pcl_lidar;
        CloudAPtr pcl_camera;

        ros::Timer timer;

        bool tf_flag;
        bool lidar_flag;
        bool camera_flag;

        string tf_filename;
};

// コンストラクタ初期化
icp_transform::icp_transform()
    : nh("~"), pcl_lidar(new CloudA), pcl_camera(new CloudA)
{
    sub_lidar  = nh.subscribe<PointCloud2> ("/lidar" , 10, &icp_transform::lidar_callback, this);
    sub_camera = nh.subscribe<PointCloud2> ("/camera", 10, &icp_transform::camera_callback, this);

    timer = nh.createTimer(ros::Duration(1.0), &icp_transform::transform,this);

    nh.param<bool>("tf_flag", tf_flag, true);
    nh.param<string>("tf_filename", tf_filename, "sq_lidar2realsense");
    lidar_flag = false;
    camera_flag = false;
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

void icp_transform::save_launch(Eigen::Matrix4d matrix)
{
    double x, y, z;
    x = matrix(0, 3);
    y = matrix(1, 3);
    z = matrix(2, 3);

    tf::Matrix3x3 mat_l;
    double roll, pitch, yaw;
    mat_l.setValue(matrix(0, 0), matrix(0, 1), matrix(0, 2),
            matrix(1, 0), matrix(1, 1), matrix(1, 2),
            matrix(2, 0), matrix(2, 1), matrix(2, 2));
    mat_l.getRPY(roll, pitch, yaw, 1);

    // Get time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);
    std::string str(buffer);

    ROS_INFO("Calibration finished succesfully...");
    ROS_INFO("x=%.4f y=%.4f z=%.4f",x,y,z);
    ROS_INFO("roll=%.4f, pitch=%.4f, yaw=%.4f", roll, pitch, yaw);
    std::string path = ros::package::getPath("calibration");
    string backuppath = path + "/launch/backup/calibrated_tf_"+ str +".launch";
    // path = path + "/launch/calibrated_tf.launch";
    path = path + "/launch/" + tf_filename +".launch";
    
    cout << endl << "Creating .launch file with calibrated TF in: "<< endl << path.c_str() << endl;
    // Create .launch file with calibrated TF
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "utf-8", "");
    doc.LinkEndChild( decl );
    TiXmlElement * root = new TiXmlElement( "launch" );
    doc.LinkEndChild( root );

    TiXmlElement * arg = new TiXmlElement( "arg" );
    arg->SetAttribute("name","stdout");
    arg->SetAttribute("default","screen");
    root->LinkEndChild( arg );

    {{{
    // string stereo_rotation = "0 0 0 -1.57079632679 0 -1.57079632679 stereo stereo_camera 10";
    // TiXmlElement * stereo_rotation_node = new TiXmlElement( "node" );
    // stereo_rotation_node->SetAttribute("pkg","tf");
    // stereo_rotation_node->SetAttribute("type","static_transform_publisher");
    // stereo_rotation_node->SetAttribute("name","stereo_ros_tf");
    // stereo_rotation_node->SetAttribute("args", stereo_rotation);
    // root->LinkEndChild( stereo_rotation_node );
    }}}

    std::ostringstream sstream;
    sstream << x << " " << y << " " << z << " " << yaw << " " <<pitch<< " " << roll << " " << cloud2_lidar.header.frame_id << " " << cloud2_camera.header.frame_id << " 100";
    // sstream << x << " " << y << " " << z << " " << yaw << " " <<pitch<< " " << roll << " stereo velodyne 100";
    string tf_args = sstream.str();
    cout << tf_args << endl;

    TiXmlElement * node = new TiXmlElement( "node" );
    node->SetAttribute("pkg","tf");
    node->SetAttribute("type","static_transform_publisher");
    node->SetAttribute("name", tf_filename);
    node->SetAttribute("args", tf_args);
    root->LinkEndChild( node );

    // Save XML file and copy
    doc.SaveFile(path);
    // doc.SaveFile(backuppath);
}



void icp_transform::transform(const ros::TimerEvent&){
  if(lidar_flag && camera_flag)
  {
      std::cout<<"lidar"<<std::endl;
      for(size_t i=0;i<pcl_lidar->points.size();i++)
          std::cout<<pcl_lidar->points[i]<<std::endl;

      std::cout<<"camera"<<std::endl;
      for(size_t i=0;i<pcl_camera->points.size();i++)
          std::cout<<pcl_camera->points[i]<<std::endl;

      pcl::IterativeClosestPoint<PointA, PointA> icp;
      icp.setInputTarget(pcl_lidar->makeShared());
      icp.setInputSource(pcl_camera->makeShared());

      CloudA Final;
      icp.align(Final);

      //変換matrixを表示する
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
      transformation_matrix = icp.getFinalTransformation ().cast<double>();
      // print4x4Matrix (transformation_matrix);
      // calc_rpy(transformation_matrix);

      tf_broadcast(transformation_matrix);
      save_launch(transformation_matrix);

      lidar_flag = false;
      camera_flag = false;
  }
}

void icp_transform::tf_broadcast(Eigen::Matrix4d matrix){
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
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), cloud2_lidar.header.frame_id, cloud2_camera.header.frame_id));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_transform");

    icp_transform icp_transform;

    ros::spin();

    return 0;
}
