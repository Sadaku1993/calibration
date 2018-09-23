#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <iostream>

using namespace std;

class TF{
    private:
        ros::NodeHandle nh;

        tf::TransformListener listener;
        tf::StampedTransform transform;

        string target_frame;
        string source_frame;

        double p_x, p_y, p_z;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ point;

    public:
        TF();
        void mainloop();
};

TF::TF()
    : nh("~")
{
    nh.param<string>("target_frame", target_frame, "centerlaser");
    nh.param<string>("source_frame", source_frame, "camera_link");
    nh.param<double>("x", p_x, 0.0);
    nh.param<double>("y", p_y, 0.0);
    nh.param<double>("z", p_z, 0.0);

    point.x = p_x;
    point.y = p_y;
    point.z = p_z;
    cloud.points.push_back(point);
}

void TF::mainloop()
{
    try{
        ros::Time time = ros::Time::now();
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(0.1));
        listener.lookupTransform(target_frame, source_frame, time, transform);

        // SHOW TF
        tf::Vector3 v = transform.getOrigin();
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        tf::Quaternion q = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll ,pitch, yaw);

        printf("x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);

        tf::Transform tf;
        tf.setOrigin(v);
        tf.setRotation(q);
        pcl::PointCloud<pcl::PointXYZ> trans_cloud;
        pcl_ros::transformPointCloud(cloud, trans_cloud, tf);

        std::cout<<"trans result"<<std::endl;
        for(size_t i=0;i<trans_cloud.points.size();i++)
            std::cout<<trans_cloud.points[i]<<std::endl;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }


}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "transform_publisher");
    
    TF tf;

    ros::Rate rate(50);

    while(ros::ok())
    {
        tf.mainloop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
