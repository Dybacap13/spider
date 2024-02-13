#include <reward.h>

int main( int argc, char **argv )
 {
    ros::init(argc, argv, "calculator_reward");
    ros::NodeHandle nh;
    Reward reward( nh);
    while (ros::ok()){
        reward.calculatorReward();
        //ros::Duration(2.0).sleep();
        ros::spinOnce();

    }

     return 0;


}
