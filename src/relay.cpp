#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    sensor_msgs::PointCloud2 pc2;
    pc2 = *msg;
    pc2.header.stamp = ros::Time::now();

    pub.publish(pc2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "relay");

    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/cloud", 10, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/relay", 10);

    ros::spin();

    return 0;
}
