## Вдохновители
<a href=https://github.com/HumaRobotics/phantomx_gazebo/tree/master>PhantomX</a>
<br> </n>
<a href=https://github.com/KevinOchs/hexapod_ros>Golem</a>



##  Nodes

**_hexapod_controller_**

Главная нода, отвечает за расчёт обратной кинематики, генерирование походки .Пока используются следующие топики:

*Subscribed Topics*

     cmd_vel (geometry_msgs/Twist)  - задание скорости движения гексапода
     state (std_msgs::Bool)  - true -робот встанет
     
*Published Topics*

    joint_states_to_gazebo (sensor_msgs::JointState) - публикует рассчитанное положение суставов.
    
**_hexapod_controller_gazebo_**

Отвечает за контроль в Gazebo

*Subscribed Topics*

     joint_states_to_gazebo (sensor_msgs::JointState) - считывает рассчитанное положение суставов и публикует в Gazebo

*Published Topics*

     /spider/j_c1_lf_position_controller/command
     /spider/j_c1_lm_position_controller/command
     /spider/j_c1_lr_position_controller/command
     /spider/j_c1_rf_position_controller/command
     /spider/j_c1_rm_position_controller/command
     /spider/j_c1_rr_position_controller/command
     /spider/j_thigh_lf_position_controller/command
     /spider/j_thigh_lm_position_controller/command
     /spider/j_thigh_lr_position_controller/command
     /spider/j_thigh_rf_position_controller/command
     /spider/j_thigh_rm_position_controller/command
     /spider/j_thigh_rr_position_controller/command
     /spider/j_tibia_lf_position_controller/command
     /spider/j_tibia_lm_position_controller/command
     /spider/j_tibia_lr_position_controller/command
     /spider/j_tibia_rf_position_controller/command
     /spider/j_tibia_rm_position_controller/command
     /spider/j_tibia_rr_position_controller/command


**_spider_description_**

Хранит параметры гексапода, его URDF-описание


**_spider_gazebo_**

     roslaunch spider_gazebo srider_gazebo.launch 
     
## Инструкция 

     roslaunch spider_gazebo srider_gazebo.launch 
     rosrun hexapod_control_gazebo hexapod_control_gazebo 
     rosrun hexapod_controller hexapod_controller 


## Суставы 

Относительно головы -->  суффиксы  

     lf   rf   --передняя левая/правая
     lm   rm   --средняя левая/правая 
     lr   rr   --задняя левая/правая 

**_Привод у туловища_**

     j_c1_lf 
     j_c1_rf 
     j_c1_lm 
     j_c1_rm 
     j_c1_lr 
     j_c1_rr 

 **_Привод колена_**
 
     j_thigh_lf
     j_thigh_rf
     j_thigh_lm
     j_thigh_rm
     j_thigh_lr
     j_thigh_rr

**_Привод у стопы_**

     j_tibia_lf_
     j_tibia_rf
     j_tibia_lm
     j_tibia_rm
     j_tibia_lr
     j_tibia_rr


