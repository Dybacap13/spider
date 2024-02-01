#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string>

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
  JointState prev_state;
  std::string name__space = "/spider/";
  std::map<std::string, ros::Publisher>
      publisher_joints_to_controller; // словарь ключ - контроллер, значение -
                                      // паблишер


int k  =0;
  int i_jointsStatesCallback = 0;
  //void jointsStatesCallback(sensor_msgs::JointState); // считывает состояние суставов
  void jointsToGazeboCallback(sensor_msgs::JointState); // считывает состояние суставов
  bool equals(sensor_msgs::JointState target_angls, JointState current_angls );
};


ControlGazebo::ControlGazebo(void)  {
  sub_joints_state = nh_.subscribe("/joints_to_gazebo", 10, &ControlGazebo::jointsToGazeboCallback, this);
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

