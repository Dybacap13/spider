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
    ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );

    legs_sub = nh_.subscribe("/move_legs/legs", 1,  &MoveFeet::legsCallback, this);
    move_feet_mode_sub = nh_.subscribe("/move_legs/mode",  1, &MoveFeet::moveFeetModeCallback, this);
    reverse_position_sub = nh_.subscribe("/move_legs/reverse_position",  1, &MoveFeet::reversePositionCallback, this);


    joint_states_sub = nh_.subscribe("/joints_to_gazebo", 1, &MoveFeet::jointStatesCallback, this);
    joint_states_pub = nh_.advertise<sensor_msgs::JointState>("/joints_to_gazebo", 1000);


    client_ = nh_.serviceClient<hexapod_msgs::Reward>(
             "/calculator_reward");
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
    current_state = msg;
    current_state_bool = true;


}


void MoveFeet::legsCallback (hexapod_msgs::MoveFeet move_feet){

    hexapod_msgs::Reward srv;
    std_msgs::Bool aaa;
    aaa.data = true;
    srv.request.request = true;
    cmd_vel = move_feet.cmd_vel;
    ROS_INFO("1");

    for (auto number_leg = 0; number_leg < move_feet.legs.size(); number_leg ++){
        last_command[number_leg] = move_feet.legs[number_leg];
    }

    std::vector<bool>reverse_command = {!last_command[0], !last_command[1], !last_command[2], !last_command[3], !last_command[4],!last_command[5]};

    interpolationOfAngles(current_state, upLegs(last_command));
    ros::Duration(0.3).sleep();

    if (!client_.call(srv)){
        std::cout << "Failed to call service /calculator_reward" << std::endl;
        interpolationOfAngles(current_state, downLegs(last_command));
        return; }

    if (srv.response.result == "wait") {
        ROS_INFO ("INIT SERVER COMPLETE");
        client_.call(srv);
    }

    //отклонились

    if (srv.response.reward_general < reward_gyroscope ){
        reward_gyroscope = srv.response.reward_general;
        interpolationOfAngles(current_state, downLegs(last_command));
        return;
    }


    //все норм
    interpolationOfAngles(current_state, moveLegs(last_command));
    ros::Duration(0.3).sleep();
    interpolationOfAngles(current_state, downLegs(last_command));
    //ros::Duration(0.3).sleep();
    interpolationOfAngles(current_state, reverseTrueLegsAndUpFalseLegs(last_command));
    ros::Duration(0.3).sleep();
    interpolationOfAngles(current_state, downLegs(reverse_command));
    reward_gyroscope = srv.response.reward_general;


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

        current_state = interpolation_angles_pub;

        std::cout <<"_________________"  <<std::endl;
        std::cout <<""  <<std::endl;
        std::cout <<""  <<std::endl;
    }

}



bool MoveFeet::comparisonJointStates(sensor_msgs::JointState first, sensor_msgs::JointState second){
    if (first.name.size() != second.name.size()) {
        ROS_ERROR("ERROR IN comparisonJointStates ");
        //ros::Duration(1.0).sleep();
        return false;
    }

    for (int i = 0; i < first.name.size(); i++ ){
        if (abs(first.position[i] - second.position[i]) > DELTA) {
            return false;
        }
    }
    return true;
}



void MoveFeet::jointStatesPublisher(sensor_msgs::JointState msg_pub){
    joint_states_pub.publish(msg_pub);
}




void MoveFeet::reversePositionCallback(std_msgs::Bool msg){
    if (!last_state_bool) {
        ROS_ERROR("NO LAST STATE");
        return;
    }
    if(msg.data) reversePosition();
}


void MoveFeet::reversePosition(){
    //ros::Duration(1.0).sleep();
    // sensor_msgs::JointState target_state;
    // sensor_msgs::JointState down_leg;
    // target_state = last_state;
    // down_leg = last_state;

    //  for (auto number_leg = 0; number_leg < 6; number_leg ++) {


    //     if (last_command[number_leg])
    //          target_state.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  + (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
    //  }

    // interpolationOfAngles(current_state, target_state);

}



sensor_msgs::JointState MoveFeet::upLegs(std::vector<bool> command){
    sensor_msgs::JointState target_state = current_state;
    std::cout <<"1) upLegs" <<std::endl;

    for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {


        if (!command[number_leg]) {
            continue;
        }


        // здесь код для ног true
        // поднимаем
        target_state.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  + (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
        std::cout <<target_state.name [number_leg * 3+1] <<" = " << target_state.position [number_leg * 3+1] <<std::endl;

     }
    //ros::Duration(2.0).sleep();

    return target_state;


}

sensor_msgs::JointState MoveFeet::moveLegs(std::vector<bool> command){
    sensor_msgs::JointState target_state = current_state;
    std::cout <<"2) moveLegs" <<std::endl;

    for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {


        if (!command[number_leg]) {
            continue;
        }



        // здесь двигаем coxa
        target_state.position[number_leg * 3] = current_state.position[number_leg * 3]  + (cmd_vel * COXA_AXIS[number_leg]);

        std::cout <<target_state.name [number_leg * 3] <<" = " << target_state.position [number_leg * 3] <<std::endl;
    }

    //ros::Duration(2.0).sleep();

    return target_state;


}

sensor_msgs::JointState MoveFeet::downLegs(std::vector<bool> command){
    sensor_msgs::JointState down_leg = current_state;
    std::cout <<"3) downLegs" <<std::endl;
    for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {
        if (command[number_leg]){
            down_leg.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  - (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
        }
        std::cout <<down_leg.name [number_leg * 3 + 1] <<" = " << down_leg.position [number_leg * 3 + 1] <<std::endl;

    }
    //ros::Duration(2.0).sleep();

    return down_leg;

}

sensor_msgs::JointState MoveFeet::reverseTrueLegsAndUpFalseLegs(std::vector<bool> command){
    sensor_msgs::JointState target_state = current_state;
    std::cout <<"4) reverseTrueLegsAndUpFalseLegs" <<std::endl;

      for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {
           if (command[number_leg]){
               target_state.position[number_leg * 3 ] = current_state.position[number_leg * 3]  - (cmd_vel * COXA_AXIS[number_leg]);
           }
           else {
               target_state.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  + (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
           }
            std::cout <<target_state.name [number_leg * 3+1] <<" = " << target_state.position [number_leg * 3+1] <<std::endl;
            std::cout <<target_state.name [number_leg * 3] <<" = " << target_state.position [number_leg * 3] <<std::endl;
      }

 // ros::Duration(2.0).sleep();
        return target_state;
}


