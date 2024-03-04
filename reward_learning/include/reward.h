
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <hexapod_msgs/Reward.h>

class Reward {
public:
  Reward(ros::NodeHandle nh_);
  std::string calculatorReward();

private:
  ros::NodeHandle nh;

  ros::Subscriber gyroscope_sub;
  ros::Subscriber odometry_sub;

  ros::Publisher reward_gyroscope_pub;
  ros::Publisher reward_odometry_pub;
  ros::Publisher reward_pub;

  ros::ServiceServer service_;

  void gyroscopeCallback(sensor_msgs::Imu gyroscope_msg);
  void odometryCallback(nav_msgs::Odometry odometry_msg);

  bool init_service(hexapod_msgs::Reward::Request &req,
                      hexapod_msgs::Reward::Response &res);

  sensor_msgs::Imu gyroscope;
  nav_msgs::Odometry odometry;
  sensor_msgs::Imu gyroscope_last;
  nav_msgs::Odometry odometry_last;

  std::vector<double> THRESHOLD_GYROSCOPE;
  double THRESHOLD_COORDINATES;
  double reward_gyroscope = 0.0;
  double reward_odometry = 0.0;
  double reward = 0.0;
  bool gyroscope_bool = false;
  bool odometry_bool = false;
  bool gyroscope_last_bool = false;
  bool odometry_last_bool = false;



};

Reward::Reward(ros::NodeHandle nh_) : nh(nh_) {
  gyroscope_sub = nh_.subscribe("/spider/gyroscope_data", 5,
                                &Reward::gyroscopeCallback, this);
  odometry_sub =
      nh_.subscribe("/odometry/calculated", 5, &Reward::odometryCallback, this);

  reward_gyroscope_pub = nh_.advertise<std_msgs::Float32>("/reward/reward_gyroscope", 1000);
  reward_odometry_pub = nh_.advertise<std_msgs::Float32>("/reward/reward_odometry", 1000);
  reward_pub = nh_.advertise<std_msgs::Float32>("/reward", 1000);

  service_ = nh_.advertiseService("/calculator_reward",
                                        &Reward::init_service, this);

  ros::param::get("THRESHOLD_GYROSCOPE", THRESHOLD_GYROSCOPE);
  ros::param::get("THRESHOLD_COORDINATES", THRESHOLD_COORDINATES);
  gyroscope_last.orientation.x = 0.0;
  gyroscope_last.orientation.y = 0.0;
  gyroscope_last.orientation.z = 0.0;

  ROS_INFO ("Reward ready");

}

bool Reward::init_service(hexapod_msgs::Reward::Request &req,
                  hexapod_msgs::Reward::Response &res){

    std::string result_reward;

    result_reward = calculatorReward();

    res.reward_general = reward;
    res.reward_odometry = reward_odometry;
    res.reward_gyroscope = reward_gyroscope;
    res.result = result_reward;
    return true;
}


void Reward::gyroscopeCallback(sensor_msgs::Imu gyroscope_msg) {
   // ROS_INFO ("AAAA");
 // gyroscope_last = gyroscope;
  gyroscope = gyroscope_msg;
  // if( !gyroscope_bool || !gyroscope_last_bool) {
  // if (gyroscope_bool) { gyroscope_last_bool = true;
  //     ROS_INFO ("gyroscope data collected");
  // }
  // }
  gyroscope_bool = true;


}

void Reward::odometryCallback(nav_msgs::Odometry odometry_msg) {

  odometry = odometry_msg;
  odometry_bool = true;
}

std::string Reward::calculatorReward() {


    if(!odometry_bool || !odometry_last_bool || !gyroscope_bool || !gyroscope_last_bool) {
        odometry_last = odometry;
        odometry_last_bool = true;
        gyroscope_last_bool = true;
        return "wait";
    }


    // расчёт одометрии
    double distance =
      sqrt(pow((odometry.pose.pose.position.x - odometry_last.pose.pose.position.x), 2) +
           pow((odometry.pose.pose.position.y - odometry_last.pose.pose.position.y), 2));

    std::cout << "odometry.pose.pose.position.x = "<< odometry.pose.pose.position.x <<std::endl;
    std::cout << "odometry.pose.pose.position.y = "<< odometry.pose.pose.position.y <<std::endl;
    std::cout << "odometry_last.pose.pose.position.x = "<< odometry.pose.pose.position.x <<std::endl;
    std::cout << "odometry_last.pose.pose.position.y = "<< odometry.pose.pose.position.y <<std::endl;
    std::cout << "distance = "<< distance <<std::endl;



    if (distance < THRESHOLD_COORDINATES)
        reward_odometry = reward_odometry - 2.0;
     else
        reward_odometry = reward_odometry + 2.0;

    std_msgs::Float32 msg;
    msg.data = reward_odometry;
    reward_odometry_pub.publish(msg);
    std::cout << "reward_odometry = " <<reward_odometry<<std::endl;

    // расчет гироскопа
    std::cout << "gyroscope.orientation.x = "<< gyroscope.orientation.x <<std::endl;
    std::cout << "gyroscope.orientation.y = "<< gyroscope.orientation.y <<std::endl;
    std::cout << "gyroscope.orientation.z = "<< gyroscope.orientation.z <<std::endl;

  if (abs(gyroscope_last.orientation.x - gyroscope.orientation.x) >
          THRESHOLD_GYROSCOPE[0] ||
      abs(gyroscope_last.orientation.y - gyroscope.orientation.y) >
          THRESHOLD_GYROSCOPE[1] ||
      abs(gyroscope_last.orientation.z - gyroscope.orientation.z) >
          THRESHOLD_GYROSCOPE[2])

      reward_gyroscope = reward_gyroscope - 2.0;
  else
      reward_gyroscope = reward_gyroscope + 2.0;


  msg.data = reward_gyroscope;
  reward_gyroscope_pub.publish(msg);
  std::cout << "reward_gyroscope = " <<reward_gyroscope<<std::endl;


  reward = reward_gyroscope + reward_odometry;
  std::cout << "reward = " <<reward<<std::endl;


  msg.data = reward;
  reward_pub.publish(msg);
  odometry_last = odometry;
  return "success";
  ros::Duration(3).sleep();
}
