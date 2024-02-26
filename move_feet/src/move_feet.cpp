#include <move_feet.h>

    // RR -> 0
    // RM -> 1
    // RF -> 2
    // LR -> 3
    // LM -> 4
    // LF -> 5


MoveFeet::MoveFeet(ros::NodeHandle nh_): nh(nh_) {


    ros::param::get( "FEMUR_ANGLE", FEMUR_ANGLE );
    ros::param::get( "FEMUR_AXIS", FEMUR_AXIS );
    ros::param::get( "COXA_AXIS", COXA_AXIS );
    ros::param::get( "INTERPOLATION_COEFFICIENT", INTERPOLATION_COEFFICIENT );
    ros::param::get( "DELTA", DELTA );

    legs_sub = nh_.subscribe("/move_legs/legs", 1,  &MoveFeet::legsCallback, this);
    move_feet_mode_sub = nh_.subscribe("/move_legs/mode",  1, &MoveFeet::moveFeetModeCallback, this);
    reverse_position_sub = nh_.subscribe("/move_legs/reverse_position",  1, &MoveFeet::moveFeetModeCallback, this);


    joint_states_sub = nh_.subscribe("/joints_to_gazebo", 1, &MoveFeet::jointStatesCallback, this);
    joint_states_pub = nh_.advertise<sensor_msgs::JointState>("/joints_to_gazebo", 1000);

    move_feet_mode = false;
    current_state_bool = false;
    ROS_INFO("MoveFeet ready");

}


void MoveFeet::moveFeetModeCallback(std_msgs::Bool msg){
    if (!current_state_bool) {
        ROS_WARN ("Wait while the current angles are calculated");
        return;
    }
    move_feet_mode = msg.data;
}


void MoveFeet::jointStatesCallback(sensor_msgs::JointState msg){
    last_state = current_state;
    current_state = msg;
    current_state_bool = true;


}


void MoveFeet::legsCallback (hexapod_msgs::MoveFeet move_feet){
    ROS_INFO("legsCallback ready");
    sensor_msgs::JointState target_state;
    sensor_msgs::JointState down_legs;
    target_state = current_state;
    down_legs =  current_state;


    for (auto number_leg = 0; number_leg < move_feet.legs.size(); number_leg ++) {

        // Если ногу не трогаем - то просто поднимаем
        if (!move_feet.legs[number_leg]) {
           continue;
        }

        //поднимаем все ноги получается
        target_state.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  + (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
        std::cout <<target_state.name [number_leg * 3+1] <<" = " << target_state.position [number_leg * 3+1] <<std::endl;



        // здесь двигаем coxa
        target_state.position[number_leg * 3] = current_state.position[number_leg * 3]  + (move_feet.cmd_vel * COXA_AXIS[number_leg]);
        down_legs.position[number_leg * 3] = current_state.position[number_leg * 3]  + (move_feet.cmd_vel * COXA_AXIS[number_leg]);

        std::cout <<target_state.name [number_leg * 3] <<" = " << target_state.position [number_leg * 3] <<std::endl;
     }

    // в итоге мы собрали все углы для перемены состояния
    // запускаем функцию плавного перехода в эти углы

    std::cout <<"____________________________" <<std::endl;

    interpolationOfAngles(current_state, target_state);
    interpolationOfAngles(target_state, down_legs);






}


void MoveFeet::interpolationOfAngles(sensor_msgs::JointState current, sensor_msgs::JointState target){

    sensor_msgs::JointState interpolation_angles_pub;
    interpolation_angles_pub = current;

    while (ros::ok()) {
        if (comparisonJointStates(target,interpolation_angles_pub )) // цикл прекратится, когда старт. значение = желаемому
            break;

        for (int i = 0; i < current_state.name.size(); i++){
            interpolation_angles_pub.position[i] = target.position[i] * INTERPOLATION_COEFFICIENT + interpolation_angles_pub.position[i] * (1 - INTERPOLATION_COEFFICIENT);
            std::cout <<interpolation_angles_pub.name [i] <<" = " << interpolation_angles_pub.position [i] <<std::endl;
        }

        jointStatesPublisher(interpolation_angles_pub);

        last_state = current_state;
        current_state = interpolation_angles_pub;

        std::cout <<"_________________"  <<std::endl;
        std::cout <<""  <<std::endl;
        std::cout <<""  <<std::endl;
    }

}



bool MoveFeet::comparisonJointStates(sensor_msgs::JointState first, sensor_msgs::JointState second){
    if (first.name.size() != second.name.size()) {
        ROS_ERROR("!!!");
        ros::Duration(1.0).sleep();
        return false;
    }

    for (int i = 0; i < first.name.size(); i++ ){
        if (abs(first.position[i] - second.position[i]) > DELTA) {
            std::cout <<"" <<std::endl;
            std::cout <<first.name[i] <<" = " << first.position[i] <<std::endl;
            std::cout << second.name[i] <<" = " << second.position[i] <<std::endl;

           // ros::Duration(1.0).sleep();
            return false;
        }
    }
    return true;
}



void MoveFeet::jointStatesPublisher(sensor_msgs::JointState msg_pub){
    ROS_INFO ("PUB");
    joint_states_pub.publish(msg_pub);
}




void MoveFeet::reversePositionCallback(std_msgs::Bool msg){
    if(!msg.data) reversePosition();
}


void MoveFeet::reversePosition(){
    interpolationOfAngles(current_state, last_state);
}
