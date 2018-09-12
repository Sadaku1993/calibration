/*
 * author : Yudai Sadakuni
 *
 * detect calibration board (camera)
 *
 */

#include <calibration/function.h>

typedef pcl::PointXYZ PointA;
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
double RADIUS;
int SIZE;
double DISTANCE;
double TORELANCE;
int MIN_SIZE, MAX_SIZE;

ros::Publisher pub_pickup;
ros::Publisher pub_plane;
ros::Publisher pub_edge;
ros::Publisher pub_edge_circle;
ros::Publisher pub_cluster;
ros::Publisher pub_centroid;
ros::Publisher pub_plane_transform;
ros::Publisher pub_edge_transform;
ros::Publisher pub_cluster_transform;
ros::Publisher pub_centroid_transform;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr input(new CloudA);
    pcl::fromROSMsg(*msg, *input);

    // Pickup Cloud
    CloudAPtr area(new CloudA);
    pickup_cloud<CloudAPtr>(input,
                            area,
                            MIN_Z,
                            MAX_Z,
                            MIN_ANG,
                            MAX_ANG,
                            MIN_DIS,
                            MAX_DIS);

    // Plane Segmentation
    CloudAPtr plane(new CloudA);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    plane_segmentation<PointA, CloudA, CloudAPtr>(area, plane, coefficients, inliers, SEGMENT_DISTANCE);
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
    string board_frame="calibration_board_camera";
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
    pub_cloud(plane, msg->header, pub_plane);
    pub_cloud(trans_cloud, msg->header, pub_plane_transform);
    pub_cloud(edge, msg->header, pub_edge_transform);
    pub_cloud(cluster_cloud, msg->header, pub_cluster_transform);
    if(flag){
       pub_cloud(transform_centroid, msg->header, pub_centroid_transform);
       pub_cloud(inverse_centroid, msg->header, pub_centroid);
    }

    printf("\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_lidar");
    ros::NodeHandle nh("~");

    // calibration board pickup param
    nh.param<double>("min_z",   MIN_Z,   -0.45);
    nh.param<double>("max_z",   MAX_Z,   1.0);
    nh.param<double>("min_ang", MIN_ANG, -0.785);
    nh.param<double>("max_ang", MAX_ANG, 0.785);
    nh.param<double>("min_dis", MIN_DIS, 1.0);
    // plane segmentation param
    nh.param<double>("seg_dis", SEGMENT_DISTANCE, 0.05);
    // 
    nh.param<double>("max_dis", MAX_DIS, 3.0);
    nh.param<double>("radius",  RADIUS,  0.05);
    nh.param<int>("size", SIZE, 70);
    nh.param<double>("distance", DISTANCE, 0.35);
    nh.param<double>("tolerance", TORELANCE, 0.03);
    nh.param<int>("min_size", MIN_SIZE, 10);
    nh.param<int>("max_size", MAX_SIZE, 100);

    ros::Subscriber sub = nh.subscribe("/cloud", 10, pcCallback);

    pub_pickup      = nh.advertise<sensor_msgs::PointCloud2>("/camera/pickup", 10);
    pub_plane       = nh.advertise<sensor_msgs::PointCloud2>("/camera/plane", 10);
    pub_edge        = nh.advertise<sensor_msgs::PointCloud2>("/camera/edge", 10);
    pub_edge_circle = nh.advertise<sensor_msgs::PointCloud2>("/camera/edge_circle", 10);
    pub_cluster     = nh.advertise<sensor_msgs::PointCloud2>("/camera/cluster", 10);
    pub_centroid    = nh.advertise<sensor_msgs::PointCloud2>("/camera/centroid", 10);
    pub_plane_transform = nh.advertise<sensor_msgs::PointCloud2>("/camera/plane/transform", 10);
    pub_edge_transform  = nh.advertise<sensor_msgs::PointCloud2>("/camera/edge/transform", 10);
    pub_cluster_transform = nh.advertise<sensor_msgs::PointCloud2>("/camera/cluster/transform", 10);
    pub_centroid_transform = nh.advertise<sensor_msgs::PointCloud2>("/camera/centroid/transform", 10);
	ros::spin();

    return 0;
}
