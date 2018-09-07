#include <ros/ros.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

using namespace std;
using namespace Eigen;

void svd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
         Eigen::Vector3f &centroid)
{
    Eigen::Matrix3f matrix;
    Eigen::Vector3f vector;

    matrix = Eigen::Matrix3f::Zero();
    vector = Eigen::Vector3f::Zero();

    for(size_t i=0;i<cloud->points.size();i++){
        pcl::PointXYZ p = cloud->points[i];
        matrix(0, 0) += p.y * p.y;
        matrix(0, 1) += p.y * p.z;
        matrix(0, 2) += p.y;
        matrix(1, 0) += p.y * p.z;
        matrix(1, 1) += p.z * p.z;
        matrix(1, 2) += p.z;
        matrix(2, 0) += p.y;
        matrix(2, 1) += p.z;
        matrix(2, 2) += 1;

        vector(0) -= (p.y * p.y * p.y + p.y * p.z * p.z);
        vector(1) -= (p.y * p.y * p.z + p.z * p.z * p.z);
        vector(2) -= (p. y* p.y + p.z * p.z);

    }

    Eigen::Vector3f ans = matrix.inverse()*vector;

    std::cout<<"maxtrix"<<"\n"<<matrix<<std::endl;
    std::cout<<"vector"<<"\n"<<vector<<std::endl;
    std::cout<<"determinant"<<"\n"<<matrix.determinant()<<std::endl;
    std::cout<<"inverse"<<"\n"<<matrix.inverse()<<std::endl;

    centroid(0) = -ans(0)/2;
    centroid(1) = -ans(1)/2;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "svd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(4);

    cloud->points[0].x=0;  cloud->points[0].y=0.0;  cloud->points[0].z=2.0;
    cloud->points[1].x=0; cloud->points[1].y=0.0;  cloud->points[1].z=-2.0;
    cloud->points[2].x=0;  cloud->points[2].y=2.0;  cloud->points[2].z=0.0;
    cloud->points[3].x=0;  cloud->points[3].y=-2.0; cloud->points[3].z=0.0;

    Eigen::Vector3f centroid;
    svd(cloud, centroid);
    std::cout<<centroid<<std::endl;
}
