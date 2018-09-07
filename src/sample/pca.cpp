#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace Eigen;

void savePCDFile(pcl::PointCloud<pcl::PointXYZ> cloud, 
                 string file_path,
                 string file_name)
{
    pcl::PointCloud<pcl::PointXYZ> save_cloud;
	pcl::copyPointCloud(cloud, save_cloud);
 
	save_cloud.width = 1;
	save_cloud.height = save_cloud.points.size();

	pcl::io::savePCDFile(file_path+file_name+".pcd", save_cloud);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void show_data(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for(size_t i=0;i<cloud->points.size();i++)
        std::cout<<"x:"<<cloud->points[i].x<<" "<<
                   "y:"<<cloud->points[i].y<<" "<<
                   "z:"<<cloud->points[i].z<<" "<<std::endl;
}
                   
int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;

    string file_path="/home/amsl/PCD/";

    // cloud.width = 5;
    // cloud.height = 1 ;
    // cloud.is_dense = true;
    cloud->points.resize(5);

    cloud->points[0].x =  0; cloud->points[0].y =  0; cloud->points[3].z = 1.0;
    cloud->points[1].x = -2; cloud->points[1].y = -2; cloud->points[3].z = -1.0;
    cloud->points[2].x =  2; cloud->points[2].y =  2; cloud->points[3].z = 1.0;
    cloud->points[3].x =  1; cloud->points[3].y = -1; cloud->points[3].z = -1.0;
    cloud->points[4].x = -1; cloud->points[4].y =  1; cloud->points[3].z = 0; 
    
    pca.setInputCloud (cloud);
    // pca.setInputCloud (cloud.makeShared ());
    
    // 共分散行列を算出
    Eigen::Matrix3f components;
    components = pca.getEigenVectors();
    std::cout<<"components"<<std::endl;
    std::cout<<components<<std::endl;

    // 係数を算出
    // Eigen::MatrixXf coefficients;
    // coefficients = pca.getCoefficients();
    // std::cout<<"coefficients"<<std::endl;
    // std::cout<<coefficients<<std::endl;

    // valueを算出
    // Eigen::Matrix3f values;
    // values = pca.getEigenVectors();
    // std::cout<<"values"<<std::endl;
    // std::cout<<values<<std::endl;

    // meanを算出
    // Eigen::Vector4f mean;
    // mean = pca.getMean();
    // std::cout<<"mean"<<std::endl;
    // std::cout<<mean<<std::endl;

    // 射影
    pcl::PointCloud<pcl::PointXYZ>::Ptr projection(new pcl::PointCloud<pcl::PointXYZ>);
    pca.project(*cloud, *projection);

    // 再構築
    pcl::PointCloud<pcl::PointXYZ>::Ptr reconstruct(new pcl::PointCloud<pcl::PointXYZ>);
    pca.reconstruct(*projection, *reconstruct);

    savePCDFile(*cloud, file_path, "cloud");
    savePCDFile(*projection, file_path, "projection");
    savePCDFile(*reconstruct, file_path, "reconstruct");

    // show data
    std::cout<<"cloud"<<std::endl;
    show_data(cloud);
    std::cout<<"projection"<<std::endl;
    show_data(projection);
    
    // viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(projection);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
