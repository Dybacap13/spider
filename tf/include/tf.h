#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>



#include <ros/package.h>
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>




#include <cstdlib>
#include <fstream>
#include <iostream>

class TFStaff {
public:
    TFStaff(ros::NodeHandle &nh_);
    void tf_calculated();
private:

    ros::NodeHandle nh;
    ros::ServiceServer isocenter_coordinates_server;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher tf_pub;



};

TFStaff::TFStaff(ros::NodeHandle &nh_)
    : nh(nh_), tfListener(tfBuffer) {
    tf_pub = nh.advertise<geometry_msgs::TransformStamped>( "/tf_calculated", 10 );
    ROS_INFO("TFStaff ready");
}


void TFStaff::tf_calculated(){
    bool is_ee_found = false;
    int attempts = 10;
    for (int i = 0; i < attempts; i++) {
      try {
        geometry_msgs::TransformStamped transform_base_from_world =
            tfBuffer.lookupTransform("world", "body_link", ros::Time(0));
        tf_pub.publish(transform_base_from_world);

        is_ee_found = true;

        break;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.1).sleep();
        continue;
      }
    }
}
