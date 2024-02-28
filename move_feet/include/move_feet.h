#include <ros/ros.h>
#include <hexapod_msgs/MoveFeet.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>




class MoveFeet {
public:
    MoveFeet(ros::NodeHandle nh_);


private:
    ros::NodeHandle nh;

    ros::Subscriber legs_sub;
    ros::Subscriber joint_states_sub;
    ros::Subscriber move_feet_mode_sub;
    ros::Subscriber reverse_position_sub;

    ros::Publisher joint_states_pub;


    sensor_msgs::JointState current_state;
    sensor_msgs::JointState target_state;
    sensor_msgs::JointState last_state;

    bool move_feet_mode = false;
    bool current_state_bool = false;
    bool last_state_bool = false;

    double FEMUR_ANGLE;
    std::vector<int> FEMUR_AXIS;
    std::vector<int> COXA_AXIS;
    double INTERPOLATION_COEFFICIENT;
    double DELTA;
    std::vector<bool> last_command = {false, false, false,false, false,false};



    void jointStatesCallback(sensor_msgs::JointState);
    void legsCallback(hexapod_msgs::MoveFeet);
    void moveFeetModeCallback(std_msgs::Bool);
    void reversePositionCallback(std_msgs::Bool msg);


    bool comparisonJointStates(sensor_msgs::JointState first, sensor_msgs::JointState second);
    void interpolationOfAngles(sensor_msgs::JointState , sensor_msgs::JointState );
    void jointStatesPublisher(sensor_msgs::JointState);
    void reversePosition();
    void downLegs();







};
