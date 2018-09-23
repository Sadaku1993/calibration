#include <ros/ros.h>
#include <ros/package.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ctime>
#include "tinyxml.h"

using namespace std;

class Transform{
    private:
        ros::NodeHandle nh;
        
        tf::TransformListener listener;
        tf::StampedTransform transform;

        string target_frame;
        string source_frame;
        string tf_filename;

    public:
        Transform();
        void mainloop();
        void save_launch(double x,
                         double y,
                         double z,
                         double roll,
                         double pitch,
                         double yaw);
};

Transform::Transform()
    :nh("~")
{
    nh.param<string>("target_frame", target_frame, "centerlaser");
    nh.param<string>("source_frame", source_frame, "camera_link");
    nh.param<string>("tf_filename" , tf_filename,  "tf_filename");
}

void Transform::save_launch(double x,
                       double y, 
                       double z,
                       double roll,
                       double pitch,
                       double yaw)
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);
    std::string str(buffer);

    std::string path = ros::package::getPath("calibration");
    path = path + "/launch/" + tf_filename +".launch";
    cout << endl << "Creating .launch file with calibrated TF in: "<< endl << path.c_str() << endl;
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "utf-8", "");
    doc.LinkEndChild( decl );
    TiXmlElement * root = new TiXmlElement( "launch" );
    doc.LinkEndChild( root );

    TiXmlElement * arg = new TiXmlElement( "arg" );
    arg->SetAttribute("name","stdout");
    arg->SetAttribute("default","screen");
    root->LinkEndChild( arg );


    std::ostringstream sstream;
    sstream << x << " " << y << " " << z << " " << yaw << " " <<pitch<< " " << roll << " " << target_frame << " " << source_frame << " 100";
    string tf_args = sstream.str();
    cout << tf_args << endl;

    TiXmlElement * node = new TiXmlElement( "node" );
    node->SetAttribute("pkg","tf");
    node->SetAttribute("type","static_transform_publisher");
    node->SetAttribute("name", tf_filename);
    node->SetAttribute("args", tf_args);
    root->LinkEndChild( node );
    doc.SaveFile(path);
}


void Transform::mainloop()
{
    try{
        ros::Time time = ros::Time::now();
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(1.0));
        listener.lookupTransform(target_frame, source_frame, time, transform);

		geometry_msgs::Transform transform_msg;
		tf::transformTFToMsg(transform, transform_msg);

        // SHOW TF
        tf::Vector3 v = transform.getOrigin();
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        tf::Quaternion q = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll ,pitch, yaw);

        printf("x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);

        save_launch(x, y, z, roll, pitch, yaw);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_saver");

    Transform tf;

    ros::Rate rate(20);

    while(ros::ok())
    {
        tf.mainloop();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
