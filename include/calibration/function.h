#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define PI 3.141592

using namespace std;
using namespace Eigen;

struct Cluster{
    float x; 
    float y; 
    float z;
    float width;
    float height;
    float depth;
    float curvature;
    Vector3f min_p;
    Vector3f max_p;
};

template<class T_C>
struct Clusters{
    Cluster data;
    T_C centroid;
    T_C points;
};

// -----------------pickup pointcloud----------------
template<class T_Ptr>
void pickup_cloud(T_Ptr cloud, 
                  T_Ptr& output,
                  double min_z,
                  double max_z,
                  double min_ang,
                  double max_ang,
                  double min_dis,
                  double max_dis)
{
    for(size_t i=0;i<cloud->points.size();i++){
        double dis = sqrt( pow(cloud->points[i].x, 2) + 
                           pow(cloud->points[i].y, 2) + 
                           pow(cloud->points[i].z, 2) );
        double theta = atan2(cloud->points[i].y, cloud->points[i].x);

        if(min_z < cloud->points[i].z && cloud->points[i].z < max_z &&
           min_ang < theta && theta < max_ang &&
           min_dis < dis && dis < max_dis)
            output->points.push_back(cloud->points[i]);
    }
}

// -----------------plane segmentation--------------------
template<class T_P, class T_C, class T_Ptr>
void plane_segmentation(T_Ptr cloud,
                        T_Ptr& plane,
                        pcl::ModelCoefficients::Ptr& coefficients,
                        pcl::PointIndices::Ptr& inliers,
                        float distance)
{
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<T_P> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }
    
    // plane pointcloud
    plane->points.resize(inliers->indices.size());
    for(size_t i=0;i<inliers->indices.size();i++)
    {
        plane->points[i] = cloud->points[inliers->indices[i]];
    }
}

// -------------------downsample---------------------
template<class T_P, class T_Ptr>
void downsampling(T_Ptr cloud,
                  T_Ptr& cloud_filtered,
                  float size)
{
    // Create the filtering object
    pcl::VoxelGrid<T_P> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (size, size, size);
    sor.filter (*cloud_filtered);
}

// ------------------edge detection-----------
template<class T_P, class T_C, class T_Ptr>
void edge_detection(T_Ptr cloud,
                    T_Ptr& cloud_edge,
                    float radius,
                    int size)
{
    // 近傍点群を探索し、cloudと重なる点群数が最も小さい中心点を算出
    pcl::KdTreeFLANN<T_P> kdtree;
    kdtree.setInputCloud (cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquareDistance;
    
    for(size_t i=0;i<cloud->points.size();i++)
    {
        T_P searchPoint = cloud->points[i];
        if(kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) >0){
            if( pointIdxRadiusSearch.size() < size ){
                cloud_edge->points.push_back(searchPoint);
            }
        }
        else
            cloud_edge->points.push_back(searchPoint);
    }
}

// --------------pickup circle edge----------------
template<class T_Ptr>
void pickup_circle_edge(T_Ptr cloud,
                        T_Ptr edge,
                        T_Ptr &circle_edge,
                        double distance)
{
    Vector3f centroid;
    centroid[0]=cloud->points[0].x;
    centroid[1]=cloud->points[0].y;
    centroid[2]=cloud->points[0].z;

    for(size_t i=1;i<cloud->points.size();i++){
        centroid[0]+=cloud->points[i].x;
        centroid[1]+=cloud->points[i].y;
        centroid[2]+=cloud->points[i].z;
    }

    double x = centroid[0]/double(cloud->points.size());
    double y = centroid[1]/double(cloud->points.size());
    double z = centroid[2]/double(cloud->points.size());

    for(size_t i=0;i<edge->points.size();i++)
    {
        double range = sqrt( pow(edge->points[i].x - x, 2) + 
                             pow(edge->points[i].y - y, 2) + 
                             pow(edge->points[i].z - z, 2) );
        if(range < distance)
            circle_edge->points.push_back(edge->points[i]);
    }
}

// --------------------transform-------------------------
void transform(Eigen::Vector3f centroid,
               pcl::ModelCoefficients::Ptr coefficients,
               tf::Transform& tf,
               string target_frame,
               string source_frame)
{
    static tf::TransformBroadcaster br;
    // 単位ベクトルを算出
    Eigen::Vector2f unit_vector;
    unit_vector(0) = coefficients->values[0]/sqrt( pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) );
    unit_vector(1) = coefficients->values[1]/sqrt( pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) );

    Eigen::Vector2f axis_vector;
    axis_vector(0) = 1;
    axis_vector(1) = 0;

    std::cout<<"unit_vector"<<unit_vector.transpose()<<std::endl;
    std::cout<<"axis_vector"<<axis_vector.transpose()<<std::endl;
 
    double yaw = acos( (unit_vector(0)*axis_vector(0)+unit_vector(1)*axis_vector(1)/(axis_vector(0)*axis_vector(0)+axis_vector(1)*axis_vector(1))) );
    std::cout<<"yaw:"<<yaw<<std::endl;
    tf::Vector3 vector(centroid(0), centroid(1), centroid(2));
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf.setOrigin(vector);
    tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), target_frame, source_frame));
}

// -----------------transform pointcloud-------------------
template<class T_Ptr>
void transform_pointcloud(T_Ptr cloud,
                          T_Ptr& trans_cloud,
                          tf::Transform transform)
{
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
}


// ------------------outlisr removal--------------------
template<class T_P, class T_Ptr>
void outlier_removal(T_Ptr cloud, T_Ptr& cloud_filtered)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<T_P> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (100);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
}

// -----------------ransac--------------------
template<class T_P, class T_Ptr>
void ransac(T_Ptr cloud, Eigen::Vector3f &centroid)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<T_P> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
        << coefficients->values[1] << " "
        << coefficients->values[2] << " " 
        << coefficients->values[3] << std::endl;
    // std::cout<<coefficients<<std::endl;
    // centroid(0) = coefficients->values[0];
    // centroid(1) = coefficients->values[1];
    // centroid(2) = coefficients->values[2];
}

// ----------------- centroid -----------------
template<class T_Ptr>
void calc_centroid(T_Ptr cloud,
                  Eigen::Vector3f &centroid)
{
    centroid(0) = cloud->points[0].x;
    centroid(1) = cloud->points[0].y;
    centroid(2) = cloud->points[0].z;
    for(size_t i=1;i<cloud->points.size();i++){
        centroid(0) += cloud->points[i].x;
        centroid(1) += cloud->points[i].y;
        centroid(2) += cloud->points[i].z;
    }
    centroid(0) /= (double)cloud->points.size();
    centroid(1) /= (double)cloud->points.size();
    centroid(2) /= (double)cloud->points.size();
}

// ----------------- least squares method -------------
template<class T_P, class T_Ptr>
void least_squares_method(T_Ptr cloud,
                          Eigen::Vector3f &centroid)
{
    Eigen::Matrix3f matrix;
    Eigen::Vector3f vector;

    matrix = Eigen::Matrix3f::Zero();
    vector = Eigen::Vector3f::Zero();

    for(size_t i=0;i<cloud->points.size();i++){
        T_P p = cloud->points[i];
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
        vector(2) -= (p.y * p.y + p.z * p.z);

    }

    Eigen::Vector3f ans = matrix.inverse()*vector;

    // std::cout<<"maxtrix"<<"\n"<<matrix<<std::endl;
    // std::cout<<"vector"<<"\n"<<vector<<std::endl;
    // std::cout<<"determinant"<<"\n"<<matrix.determinant()<<std::endl;
    // std::cout<<"inverse"<<"\n"<<matrix.inverse()<<std::endl;

    centroid(0) = -ans(0)/2;
    centroid(1) = -ans(1)/2;
    centroid(2) = sqrt(centroid(0)*centroid(0)+
                       centroid(1)*centroid(1)-
                       ans(2));
}


// ------------------Clustering--------------------

template<class T_C>
void getClusterInfo(T_C pt, Cluster& cluster)
{
    Vector3f centroid;
    centroid[0]=pt.points[0].x;
    centroid[1]=pt.points[0].y;
    centroid[2]=pt.points[0].z;

    Vector3f min_p;
    min_p[0]=pt.points[0].x;
    min_p[1]=pt.points[0].y;
    min_p[2]=pt.points[0].z;

    Vector3f max_p;
    max_p[0]=pt.points[0].x;
    max_p[1]=pt.points[0].y;
    max_p[2]=pt.points[0].z;

    for(size_t i=1;i<pt.points.size();i++){
        centroid[0]+=pt.points[i].x;
        centroid[1]+=pt.points[i].y;
        centroid[2]+=pt.points[i].z;
        if (pt.points[i].x<min_p[0]) min_p[0]=pt.points[i].x;
        if (pt.points[i].y<min_p[1]) min_p[1]=pt.points[i].y;
        if (pt.points[i].z<min_p[2]) min_p[2]=pt.points[i].z;

        if (pt.points[i].x>max_p[0]) max_p[0]=pt.points[i].x;
        if (pt.points[i].y>max_p[1]) max_p[1]=pt.points[i].y;
        if (pt.points[i].z>max_p[2]) max_p[2]=pt.points[i].z;
    }

    cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;
}

template<class T_P, class T_C, class T_Ptr>
void clustering(T_Ptr cloud_in,
                vector< Clusters<T_C> >& cluster_array,
                double tolerance,
                double min_size,
                double max_size){
    //downsampled point's z =>0
    vector<float> tmp_z;
    tmp_z.resize(cloud_in->points.size());
	for(int i=0;i<(int)cloud_in->points.size();i++){
        tmp_z[i]=cloud_in->points[i].z;
		cloud_in->points[i].z  = 0.0;
    }

    typename pcl::search::KdTree<T_P>::Ptr tree(new pcl::search::KdTree<T_P>);

    tree->setInputCloud (cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T_P> ec;
    ec.setClusterTolerance (tolerance); // 15cm
    ec.setMinClusterSize (min_size);
    ec.setMaxClusterSize (max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud_in);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)cloud_in->points.size();i++)
        cloud_in->points[i].z=tmp_z[i];

    for(int iii=0;iii<(int)cluster_indices.size();iii++)
    {
        // cluster points
        T_Ptr cloud_cluster (new T_C);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        // cluster data
        Cluster data;
        for(int jjj=0;jjj<int(cluster_indices[iii].indices.size());jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj] = cloud_in->points[p_num];
        }
        getClusterInfo(*cloud_cluster, data);

		T_P center;
		center.x = data.x;
		center.y = data.y;
		center.z = data.z;

        Clusters<T_C> cluster;
        cluster.data = data;
        cluster.centroid.points.push_back(center);
        for(size_t i=0;i<cloud_cluster->points.size();i++)
            cluster.points.points.push_back(cloud_cluster->points[i]);

        cluster_array.push_back(cluster);
    }
}


//---------------circle area---------------------------
template<class T_P, class T_C, class T_Ptr>
void circle_area(T_Ptr cloud,
                 T_Ptr circle,
                 double param,
                 vector< Clusters<T_C> >& cluster_array)
{
    Cluster cluster;
    getClusterInfo(*cloud, cluster);

    int position;
    if(-param<cluster.y && cluster.y<param){
        position = 0;  // center
        std::cout<<"center:"<<cluster.y<<std::endl;
    }
    else if(param<cluster.y){
        position = 1;   // left
        std::cout<<"left"<<cluster.y<<std::endl;
    }
    else if(cluster.y<param){
        position = 2;  // right
        std::cout<<"right"<<cluster.y<<std::endl;
    }
    else 
        return;

    Clusters<T_C> cluster_1, cluster_2, cluster_3, cluster_4;
    if(position==0){
        for(size_t i=0;i<circle->points.size();i++)
        {
            T_P pt = circle->points[i];
            if(cluster.y<pt.y && cluster.z<pt.z)
                cluster_1.points.push_back(pt);
            else if(cluster.y<pt.y && pt.z<cluster.z)
                cluster_2.points.push_back(pt);
            else if(pt.y<cluster.y && pt.z<cluster.z)
                cluster_3.points.push_back(pt);
            else
                cluster_4.points.push_back(pt);
        }
    }
    else if(position==1){
        for(size_t i=0;i<circle->points.size();i++)
        {
            T_P pt=circle->points[i];
            if(pt.x<cluster.x && cluster.z<pt.z)
                cluster_1.points.push_back(pt);
            else if(pt.x<cluster.x && pt.z<cluster.z)
                cluster_2.points.push_back(pt);
            else if(cluster.x<pt.x && pt.z<cluster.z)
                cluster_3.points.push_back(pt);
            else
                cluster_4.points.push_back(pt);
        }
    }
    else if(position==2){
        for(size_t i=0;i<circle->points.size();i++)
        {
            T_P pt=circle->points[i];
            if(cluster.x<pt.x && cluster.z<pt.z)
                cluster_1.points.push_back(pt);
            else if(cluster.x<pt.x && pt.z<cluster.z)
                cluster_2.points.push_back(pt);
            else if(pt.x<cluster.x && pt.z<cluster.z)
                cluster_3.points.push_back(pt);
            else 
                cluster_4.points.push_back(pt);
        }
    }

    cluster_array.push_back(cluster_1);
    cluster_array.push_back(cluster_2);
    cluster_array.push_back(cluster_3);
    cluster_array.push_back(cluster_4);
}

// ---------------pca---------------
template<class T_P, class T_C, class T_Ptr>
void principal_component_analysis(T_Ptr cloud, 
                                  pcl::PCA<T_P>& pca,
                                  T_Ptr &projection)
{
    pca.setInputCloud(cloud);
    // pca.setInputCloud(cloud.makeShared());
    // makeShared()はポインタにアクセスできるようになる便利な奴
    pca.project(*cloud, *projection);

    // show pca infomation
    Eigen::Matrix3f components;
    components = pca.getEigenVectors();
    std::cout<<"components"<<std::endl;
    std::cout<<components<<std::endl;
    SelfAdjointEigenSolver<Matrix3f> es(components);
    std::cout<<"The eigenvalues of components : \n"<<es.eigenvalues()<<std::endl; // 固有値はベクトルの形で得られる
    std::cout<<"The eigenvectors of components : \n"<<es.eigenvectors()<<std::endl; // 固有ベクトルは行列の形で得られる
    float min_eigen = es.eigenvalues()(0);
    std::cout<<"The minimun eigenvalue = " << min_eigen << std::endl;
    Eigen::Vector3f min_eigen_vector = es.eigenvectors().col(0);
    std::cout<<"the eigenvector corresponding the minimum eigenvalue = " << min_eigen_vector.transpose() << std::endl; 

}

// ---------------pca clustering-------------
template<class T_P, class T_C, class T_Ptr>
void pca_clustering(T_Ptr cloud,
                    T_Ptr projection,
                    pcl::PCA<T_P> pca,
                    vector< Clusters<T_C> >& cluster_array)
{
    Clusters<T_C> cluster_0, cluster_1, cluster_2, cluster_3;

    for(size_t i=0;i<projection->points.size();i++)
    {
        if(0<projection->points[i].x && 0<projection->points[i].y)
            cluster_0.points.points.push_back(cloud->points[i]);
        else if(projection->points[i].x<0 && 0<projection->points[i].y)
            cluster_1.points.points.push_back(cloud->points[i]);
        else if(projection->points[i].x<0 && projection->points[i].y<0)
            cluster_2.points.points.push_back(cloud->points[i]);
        else 
            cluster_3.points.points.push_back(cloud->points[i]);
    }
    
    cluster_array.push_back(cluster_0);
    cluster_array.push_back(cluster_1);
    cluster_array.push_back(cluster_2);
    cluster_array.push_back(cluster_3);
}

template<class T_Ptr>
void pub_cloud(T_Ptr cloud, 
               std_msgs::Header header, 
               ros::Publisher pub)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = header.frame_id;
    pub.publish(output);
}
