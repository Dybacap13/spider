#include <ros/ros.h>
#include <nav_msgs/Odometry>
#include <sensor_msgs/Imu>

class Reward {
public:
    Reward(ros::NodeHandle nh_);

private:
    ros::NodeHandle nh;

    ros::Subscriber gyroscope_sub;
    ros::Subscriber odometry_sub;

    ros::Publisher reward_pub;


    void gyroscopeCallback(sensor_msgs::Imu gyroscope_msg);
    void odometryCallback(nav_msgs::Odometry odometry_msg);
    void calculatorReward();

    sensor_msgs::Imu gyroscope;
    nav_msgs::Odometry odometry;
    sensor_msgs::Imu gyroscope_last;
    nav_msgs::Odometry odometry_last;

};

Reward::Reward(ros::NodeHandle nh_)
    : nh(nh_){
    gyroscope_sub = nh_.subscribe("/spider/gyroscope_data", 5,  &Reward::gyroscopeCallback, this);
    odometry_sub = nh_.subscribe("/odometry/calculated", 5,  &Reward::odometryCallback, this);

    reward_pub = nh_.advertise<std_msgs::Float>("/reward", 1000);

}

void Reward::gyroscopeCallback(sensor_msgs::Imu gyroscope_msg){
    gyroscope_last = gyroscope;
    gyroscope = gyroscope_msg;
}

void Reward::odometryCallback(nav_msgs::Odometry odometry_msg){
    odometry_last = odometry;
    odometry = odometry_msg;
}


void Reward::calculatorReward(){

}
