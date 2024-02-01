#include <ros/ros.h>

#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/MoveFeet.h>


class MoveFeet {
public:
    MoveFeet(ros::NodeHandle nh_);

    bool body_yes;
    bool feet_yes;
    bool legs_yes;


    ros::NodeHandle nh;
    hexapod_msgs::Pose body_;
    hexapod_msgs::FeetPositions feet_;
    hexapod_msgs::LegsJoints legs_;
    hexapod_msgs::MoveFeet move_feet_;




    ros::Subscriber body_sub;
    ros::Subscriber feet_position_sub;
    ros::Subscriber legs_sub;
    ros::Subscriber move_feet_sub;

    ros::Publisher move_feet_pub;

    void bodyCallback (hexapod_msgs::Pose body);
    void feetPositionCallback (hexapod_msgs::FeetPositions feet);
    void legsCallback(hexapod_msgs::LegsJoints legs);
    void moveFeetCallback(hexapod_msgs::MoveFeet move_feet);



};
