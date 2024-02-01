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
}


void ControlGazebo::legToGazeboCallback(hexapod_msgs::MoveFeet move_feet){
    std_msgs::Float64 coxa;
    std_msgs::Float64 femur;
    std_msgs::Float64 tibia;

    coxa.data = move_feet.position_leg.coxa;
    femur.data = move_feet.position_leg.femur;
    tibia.data = move_feet.position_leg.tibia;

    switch (move_feet.number_leg) {
        case 0:
            publisher_joints_to_controller[joints_names[0]].publish(coxa);
            publisher_joints_to_controller[joints_names[1]].publish(femur);
            publisher_joints_to_controller[joints_names[2]].publish(tibia);
            break;
        case 1:
            publisher_joints_to_controller[joints_names[3]].publish(coxa);
            publisher_joints_to_controller[joints_names[4]].publish(femur);
            publisher_joints_to_controller[joints_names[5]].publish(tibia);
            break;
        case 2:
            publisher_joints_to_controller[joints_names[6]].publish(coxa);
            publisher_joints_to_controller[joints_names[7]].publish(femur);
            publisher_joints_to_controller[joints_names[8]].publish(tibia);
            break;
        case 3:
            publisher_joints_to_controller[joints_names[9]].publish(coxa);
            publisher_joints_to_controller[joints_names[10]].publish(femur);
            publisher_joints_to_controller[joints_names[11]].publish(tibia);
            break;
        case 4:
            publisher_joints_to_controller[joints_names[12]].publish(coxa);
            publisher_joints_to_controller[joints_names[13]].publish(femur);
            publisher_joints_to_controller[joints_names[14]].publish(tibia);
            break;
        case 5:
            publisher_joints_to_controller[joints_names[15]].publish(coxa);
            publisher_joints_to_controller[joints_names[16]].publish(femur);
            publisher_joints_to_controller[joints_names[17]].publish(tibia);
            break;

        default:
            std::cout << "Unknown number legs " << move_feet.number_leg << std::endl;


    }

}

