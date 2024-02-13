
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

class Reward {
public:
  Reward(ros::NodeHandle nh_);
  void calculatorReward();

private:
  ros::NodeHandle nh;

  ros::Subscriber gyroscope_sub;
  ros::Subscriber odometry_sub;

  ros::Publisher reward_pub;

  void gyroscopeCallback(sensor_msgs::Imu gyroscope_msg);
  void odometryCallback(nav_msgs::Odometry odometry_msg);


  sensor_msgs::Imu gyroscope;
  nav_msgs::Odometry odometry;
  sensor_msgs::Imu gyroscope_last;
  nav_msgs::Odometry odometry_last;

  std::vector<double> THRESHOLD_GYROSCOPE;
  double THRESHOLD_COORDINATES;
  int reward;
};

Reward::Reward(ros::NodeHandle nh_) : nh(nh_) {
  gyroscope_sub = nh_.subscribe("/spider/gyroscope_data", 5,
                                &Reward::gyroscopeCallback, this);
  odometry_sub =
      nh_.subscribe("/odometry/calculated", 5, &Reward::odometryCallback, this);

  reward_pub = nh_.advertise<std_msgs::Float32>("/reward", 1000);

  ros::param::get("THRESHOLD_GYROSCOPE", THRESHOLD_GYROSCOPE);
  ros::param::get("THRESHOLD_COORDINATES", THRESHOLD_COORDINATES);
  reward = 0;
}

void Reward::gyroscopeCallback(sensor_msgs::Imu gyroscope_msg) {
  gyroscope_last = gyroscope;
  gyroscope = gyroscope_msg;
}

void Reward::odometryCallback(nav_msgs::Odometry odometry_msg) {
  odometry_last = odometry;
  odometry = odometry_msg;
}

void Reward::calculatorReward() {
  double distance =
      sqrt(pow((odometry.pose.pose.position.x - odometry_last.pose.pose.position.x), 2) +
           pow((odometry.pose.pose.position.y - odometry_last.pose.pose.position.y), 2));

  // мы не прошли порог
  if (distance < THRESHOLD_COORDINATES) {
    reward = reward - 2;
  } else
    reward = reward + 2;


  // мы отклонились
  if (abs(gyroscope_last.orientation.x - gyroscope.orientation.x) >
          THRESHOLD_GYROSCOPE[0] &&
      abs(gyroscope_last.orientation.y - gyroscope.orientation.y) >
          THRESHOLD_GYROSCOPE[1] &&
      abs(gyroscope_last.orientation.z - gyroscope.orientation.z) >
          THRESHOLD_GYROSCOPE[2]) {

      reward = reward - 2;
  }
  else reward = reward + 2;
  std_msgs::Float32 msg;
  msg.data = reward;
  reward_pub.publish(msg);
  ros::Duration(3).sleep();
}
