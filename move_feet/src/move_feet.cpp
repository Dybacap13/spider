#include <move_feet.h>



    // RR -> 0
    // RM -> 1
    // RF -> 2
    // LR -> 3
    // LM -> 4
    // LF -> 5



MoveFeet::MoveFeet(ros::NodeHandle nh_): nh(nh_) {

    //body_sub = nh_.subscribe("/body_position", 5,  &MoveFeet::bodyCallback, this);
    //feet_position_sub = nh_.subscribe("/feet_position", 5,  &MoveFeet::feetPositionCallback, this);
    legs_sub = nh_.subscribe("/legs", 5,  &MoveFeet::legsCallback, this);
    move_feet_sub = nh_.subscribe("/move_feet", 5,  &MoveFeet::moveFeetCallback, this);

    move_feet_pub = nh_.advertise<hexapod_msgs::MoveFeet>("/control_gazebo_legs", 1000);

    bool body_yes = false;
    bool feet_yes = false;
    bool legs_yes = false;


}


void MoveFeet::legsCallback (hexapod_msgs::LegsJoints legs){
    legs_ = legs;
    legs_yes = true;
}


void MoveFeet::moveFeetCallback (hexapod_msgs::MoveFeet move_feet){
    if (legs_yes){
        ROS_ERROR ("Position legs not init");
        return;
    }
    hexapod_msgs::MoveFeet feet_pub;
    feet_pub.number_leg = move_feet.number_leg;
    feet_pub.position_leg = legs_.leg[move_feet.number_leg];

    move_feet_pub.publish(feet_pub);


}


