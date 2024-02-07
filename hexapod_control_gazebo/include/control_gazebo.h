#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string>
#include <hexapod_msgs/MoveFeet.h>

#include <ctime>
#include <iostream>
#include <map>
#include <typeinfo>
#include <vector>

float DELTA = 0.01;


std::vector<std::string> joints_names = {
    "j_c1_rr",
    "j_thigh_rr" ,
    "j_tibia_rr" ,
    "j_c1_rm" ,
    "j_thigh_rm" ,
    "j_tibia_rm" ,
    "j_c1_rf" ,
    "j_thigh_rf" ,
    "j_tibia_rf" ,
    "j_c1_lr" ,
    "j_thigh_lr" ,
    "j_tibia_lr" ,
    "j_c1_lm",
    "j_thigh_lm" ,
    "j_tibia_lm" ,
    "j_c1_lf" ,
    "j_thigh_lf" ,
    "j_tibia_lf" ,

};

struct JointState {
  std::vector<std::string> joints;
  std::vector<double> angles;
  bool yes = false;
};

class ControlGazebo {
public:
  ControlGazebo( void );
  ~ControlGazebo() = default;

  JointState joints_state_node;
  void phublisherJointsStates();

private:
  ros::Subscriber sub_joints_state;
  ros::Publisher pub_in_controller;
  ros::NodeHandle nh_;
  ros::Subscriber leg_gazebo_sub;
  sensor_msgs::JointState current_state;



  std::string name__space = "/spider/";
  std::map<std::string, ros::Publisher>
      publisher_joints_to_controller; // словарь ключ - контроллер, значение -
                                      // паблишер



  void jointsToGazeboCallback(sensor_msgs::JointState); // считывает состояние суставов
  void legToGazeboCallback(hexapod_msgs::MoveFeet move_feet);

};


ControlGazebo::ControlGazebo(void)  {
  sub_joints_state = nh_.subscribe("/joints_to_gazebo", 10, &ControlGazebo::jointsToGazeboCallback, this);
  leg_gazebo_sub = nh_.subscribe("/control_gazebo_legs", 10, &ControlGazebo::legToGazeboCallback, this);

   for (int i = 0; i < 18; i++) {
     pub_in_controller = nh_.advertise<std_msgs::Float64>(name__space + joints_names[i] + "_position_controller/command", 1000);
     publisher_joints_to_controller[joints_names[i]] = pub_in_controller;
     std::cout <<  name__space + joints_names[i] + "_position_controller/command" << std::endl;
   }
}



void ControlGazebo::jointsToGazeboCallback(sensor_msgs::JointState msg_joint_state){
    std_msgs::Float64 msg;
    for(int i = 0 ; i < 18; i++){
    msg.data = msg_joint_state.position[i] * (-1);
    std::cout <<  msg_joint_state.name[i] <<" = "<< msg_joint_state.position[i] << std::endl;
    publisher_joints_to_controller[joints_names[i]].publish(msg);

    }
    std::cout <<  "__________________________" << std::endl;
    current_state =  msg_joint_state;
}


void ControlGazebo::legToGazeboCallback(hexapod_msgs::MoveFeet move_feet){

    std_msgs::Float64 coxa;
    std_msgs::Float64 femur;
    std_msgs::Float64 tibia;
    int sing = -1;

    if (current_state.position[move_feet.number_leg * 3 + 1] < 0 ) int sing = 1;

    coxa.data = current_state.position[move_feet.number_leg * 3] * (-1) + move_feet.cmd_vel;
    femur.data = current_state.position[move_feet.number_leg * 3 + 1] * (-1) +  ( 0.2 * sing);
    tibia.data = current_state.position[move_feet.number_leg * 3 + 2] * (-1);


    publisher_joints_to_controller[joints_names[move_feet.number_leg * 3 + 1]].publish(femur);
    publisher_joints_to_controller[joints_names[move_feet.number_leg * 3]].publish(coxa);
    publisher_joints_to_controller[joints_names[move_feet.number_leg * 3 + 2]].publish(tibia);

    femur.data = femur.data -  (0.2 * sing);
    publisher_joints_to_controller[joints_names[move_feet.number_leg * 3 + 1]].publish(femur);

}

