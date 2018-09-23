/* 
 * author : Yudai Sadakuni
 *
 * detect calibration board (lidar)
 *
 */

#include <calibration/function.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <calibration/calibration_lidarConfig.h>

typedef pcl::PointXYZI PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

// このパラメータで取得する点群のエリアを確定する
// キャリブレーションボードが収まる範囲に調整すること
double MIN_Z, MAX_Z;
double MIN_DIS, MAX_DIS;
double MIN_ANG, MAX_ANG;

// segmentation用
double SEGMENT_DISTANCE;

// edge detection用
// double RADIUS;
// int SIZE;
// double DISTANCE;
// double TORELANCE;
// int MIN_SIZE, MAX_SIZE;

ros::Publisher pub_pickup;
ros::Publisher pub_outlier;
ros::Publisher pub_plane;
ros::Publisher pub_plane_transform;
ros::Publisher pub_edge;
ros::Publisher pub_edge_transform;
ros::Publisher pub_cluster;
ros::Publisher pub_cluster_transform;
ros::Publisher pub_centroid;
ros::Publisher pub_centroid_transform;

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
    
    if(area->points.size()==0)
        return;

    // outlier removal
    CloudAPtr outlier(new CloudA);
    outlier_removal<PointA, CloudAPtr>(area, outlier, 100, 1.0);
    
    if(outlier->points.size()==0)
        return;

    // Plane Segmentation
    CloudAPtr plane(new CloudA);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    plane_segmentation<PointA, CloudA, CloudAPtr>(outlier, plane, coefficients, inliers, SEGMENT_DISTANCE);
    std::cout<<"coefficients "<<
                coefficients->values[0]<<" "<<
                coefficients->values[1]<<" "<<
                coefficients->values[2]<<" "<<
                coefficients->values[3]<<std::endl;

    if(plane->points.size()==0)
        return;

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

    // edge detection
    CloudAPtr edge(new CloudA);
    // grid_base_edge_detection<PointA, CloudA, CloudAPtr>(trans_cloud, edge, 0.09);
    side_edge_detection<PointA, CloudA, CloudAPtr>(trans_cloud, edge, 0.05);
    vertical_edge_detection<PointA, CloudA, CloudAPtr>(trans_cloud, edge, 0.05);

    // clustering
    vector<Clusters <CloudA> > clusters_transform;
    circle_clustering<PointA, CloudA, CloudAPtr>(trans_cloud, edge, clusters_transform);
    CloudAPtr cluster_cloud(new CloudA);
    for(size_t i=0;i<clusters_transform.size();i++)
        *cluster_cloud += clusters_transform[i].points;

    // least_squares_method
    CloudAPtr transform_centroid(new CloudA);
    for(size_t i=0;i<clusters_transform.size();i++){
        Eigen::Vector3f centroid;
        least_squares_method<PointA, CloudAPtr>(clusters_transform[i].points.makeShared(), centroid);
        PointA centroid_point;
        centroid_point.x = 0;
        centroid_point.y = centroid(0);
        centroid_point.z = centroid(1);
        transform_centroid->points.push_back(centroid_point);
        std::cout<<"centroid : "<<centroid_point<<std::endl;
    }

    // checker
    bool flag;
    flag = checker<CloudAPtr>(transform_centroid, 5.0);
    
    // centroid
    CloudAPtr inverse_centroid(new CloudA);
    transform_pointcloud(transform_centroid, inverse_centroid, tf);

    pub_cloud(area, msg->header, pub_pickup);
    pub_cloud(outlier,  msg->header, pub_outlier);
    pub_cloud(plane, msg->header, pub_plane);
    pub_cloud(trans_cloud, msg->header, pub_plane_transform);
    pub_cloud(edge, msg->header, pub_edge_transform);
    pub_cloud(cluster_cloud, msg->header, pub_cluster_transform);
    if(flag){
       pub_cloud(transform_centroid, msg->header, pub_centroid_transform);
       pub_cloud(inverse_centroid, msg->header, pub_centroid);
    }

    printf("\n");

    {{{
    /*
    // edge detection
    CloudAPtr edge(new CloudA);
    edge_detection<PointA, CloudA, CloudAPtr>(outlier, edge, RADIUS, SIZE);

    // pickup circle edge
    CloudAPtr circle_edge(new CloudA);
    pickup_circle_edge(outlier, edge, circle_edge, DISTANCE);

    // cluster circle
    vector<Clusters <CloudA> > clusters;
    circle_area<PointA, CloudA, CloudAPtr>(outlier, circle_edge, 0.5, clusters);

    CloudAPtr cluster(new CloudA);
    for(size_t i=0;i<clusters.size();i++)
         *cluster += clusters[i].points;

    // plane centroid
    Eigen::Vector3f plane_centroid;
    calc_centroid<CloudAPtr>(outlier, plane_centroid);
    std::cout<<"plane centroid "<<plane_centroid.transpose()<<std::endl;

    // Plane Segmentation
    CloudAPtr plane(new CloudA);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    plane_segmentation<PointA, CloudA, CloudAPtr>(outlier, plane, coefficients, inliers, 0.5);
    std::cout<<"coefficients "<<
                coefficients->values[0]<<" "<<
                coefficients->values[1]<<" "<<
                coefficients->values[2]<<" "<<
                coefficients->values[3]<<std::endl;
    
    // transform
    string laser_frame=msg->header.frame_id;
    string board_frame="calibration_board";
    tf::Transform tf;
    transform(plane_centroid, coefficients, tf, laser_frame, board_frame);

    // transform pointcloud
    CloudAPtr trans_cloud(new CloudA);
    transform_pointcloud(outlier, trans_cloud, tf.inverse());

    // circle transform pointcloud
    vector<Clusters <CloudA> > clusters_transform;
    for(size_t i=0;i<clusters.size();i++){
        Clusters<CloudA> cluster_transform;
        CloudAPtr pt(new CloudA);
        transform_pointcloud(clusters[i].points.makeShared(), pt, tf.inverse());
        cluster_transform.points += *pt;
        clusters_transform.push_back(cluster_transform);
    }

    CloudAPtr trans_circle(new CloudA);
    for(size_t i=0;i<clusters_transform.size();i++)
        *trans_circle += clusters_transform[i].points;

    // least_squares_method
    CloudAPtr transform_centroid(new CloudA);
    for(size_t i=0;i<clusters_transform.size();i++){
        Eigen::Vector3f centroid;
        least_squares_method<PointA, CloudAPtr>(clusters_transform[i].points.makeShared(), centroid);
        PointA centroid_point;
        centroid_point.x = 0;
        centroid_point.y = centroid(0);
        centroid_point.z = centroid(1);
        transform_centroid->points.push_back(centroid_point);
        std::cout<<"centroid : "<<centroid_point<<std::endl;
    }

    // centroid
    CloudAPtr inverse_centroid(new CloudA);
    transform_pointcloud(transform_centroid, inverse_centroid, tf);
    
    pub_cloud(area, msg->header, pub_pickup);
    pub_cloud(outlier,  msg->header, pub_outlier);
    pub_cloud(edge, msg->header, pub_edge);
    pub_cloud(circle_edge, msg->header, pub_edge_circle); 
    pub_cloud(cluster, msg->header, pub_cluster);
    pub_cloud(trans_cloud, msg->header, pub_transform);
    pub_cloud(trans_circle, msg->header, pub_transform_circle);
    pub_cloud(transform_centroid, msg->header, pub_transform_centroid);
    pub_cloud(inverse_centroid, msg->header, pub_centroid);

    */
    }}}

    {{{
    // centroid
    // CloudAPtr centroid(new CloudA);
    // for(size_t i=0;i<clusters_transform.size();i++){
    //     Eigen::Vector3f vector;
    //     calc_centroid<CloudAPtr>(clusters_transform[i].points.makeShared(), vector);
    //     PointA pt;
    //     pt.x = vector(0);
    //     pt.y = vector(1);
    //     pt.z = vector(2);
    //     std::cout<<vector<<std::endl;
    //     centroid->points.push_back(pt);
    // }


    // pca
    // CloudAPtr projection(new CloudA);
    // pcl::PCA<PointA> pca;
    // principal_component_analysis<PointA, CloudA, CloudAPtr>(circle_edge, pca, projection);

    // pca clustering
    // vector<Clusters <CloudA> > clusters;
    // pca_clustering(circle_edge, projection, pca, clusters);

    // clustering
    // vector<Clusters <CloudA> > clusters;
    // clustering<PointA, CloudA, CloudAPtr>(circle_edge, clusters, TORELANCE, MIN_SIZE, MAX_SIZE);
    // cout<<"size:"<<clusters.size()<<endl;
    // CloudAPtr cluster(new CloudA);
    // for(size_t i=0;i<clusters.size();i++)
    //     *cluster += clusters[i].points;
    }}}

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
    nh.param<double>("max_z",   MAX_Z,   1.0);
    nh.param<double>("min_ang", MIN_ANG, -0.785);
    nh.param<double>("max_ang", MAX_ANG, 0.785);
    nh.param<double>("min_dis", MIN_DIS, 1.0);
    nh.param<double>("max_dis", MAX_DIS, 3.0);
    nh.param<double>("seg_dis", SEGMENT_DISTANCE, 0.05);
    // nh.param<double>("radius",  RADIUS,  0.05);
    // nh.param<int>("size", SIZE, 70);
    // nh.param<double>("distance", DISTANCE, 0.35);
    // nh.param<double>("tolerance", TORELANCE, 0.03);
    // nh.param<int>("min_size", MIN_SIZE, 10);
    // nh.param<int>("max_size", MAX_SIZE, 100);

    ros::Subscriber sub = nh.subscribe("/cloud", 10, pcCallback);

    // dynamic reconfigure
    dynamic_reconfigure::Server<calibration::calibration_lidarConfig> server;
    dynamic_reconfigure::Server<calibration::calibration_lidarConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    pub_pickup   = nh.advertise<sensor_msgs::PointCloud2>("/cloud/pickup", 10);
    pub_outlier  = nh.advertise<sensor_msgs::PointCloud2>("/cloud/outlier", 10);
    pub_plane       = nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 10);
    pub_plane_transform = nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane/transform", 10);
    pub_edge     = nh.advertise<sensor_msgs::PointCloud2>("/cloud/edge", 10);
    pub_edge_transform     = nh.advertise<sensor_msgs::PointCloud2>("/cloud/edge/transform", 10);
    pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/cloud/cluster", 10);
    pub_cluster_transform = nh.advertise<sensor_msgs::PointCloud2>("/cloud/cluster/transform", 10);
    pub_centroid = nh.advertise<sensor_msgs::PointCloud2>("/cloud/centroid", 10);
    pub_centroid_transform = nh.advertise<sensor_msgs::PointCloud2>("/cloud/centroid/transform", 10);
    
    ros::spin();

    return 0;
}
