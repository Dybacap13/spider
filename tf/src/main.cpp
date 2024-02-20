#include <tf.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_node");

  ros::NodeHandle nh;

  TFStaff tf_staff(
      nh);

  while (ros::ok()){
      tf_staff.tf_calculated();
      ros::Duration(0.001).sleep();

  }

  return 0;
}
