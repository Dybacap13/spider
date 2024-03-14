#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <matrix.h>



class ReinforcementLearning {

public:
    ReinforcementLearning(ros::NodeHandle nh_);
    void algoritm();


private:
    int NUMBER_NEURON;
    double VOLTAGE_TRESHOLD;
    double DELTA;
    double LEARNING_RATE;
    double LEAK_RATE;
    int TIME_ITERATION;
    int ITERATION_SPIKE_IN;


    ros::NodeHandle nh;
    ros::Subscriber reward_sub;
    void rewardCallback(std_msgs::Float32 msg);



    double reward = 0.0;
    double reward_last = 0.0;

};




ReinforcementLearning::ReinforcementLearning(ros::NodeHandle nh_) : nh(nh_) {
  reward_sub = nh_.subscribe("/reward", 5, &ReinforcementLearning::rewardCallback, this);

  ros::param::get("NUMBER_NEURON", NUMBER_NEURON);
  ros::param::get("VOLTAGE_TRESHOLD", VOLTAGE_TRESHOLD);
  ros::param::get("DELTA", DELTA);
  ros::param::get("LEARNING_RATE", LEARNING_RATE);
  ros::param::get("LEAK_RATE", LEAK_RATE);
  ros::param::get("TIME_ITERATION", TIME_ITERATION);
  ros::param::get("ITERATION_SPIKE_IN", ITERATION_SPIKE_IN);

}
void ReinforcementLearning::rewardCallback(std_msgs::Float32 msg){
    reward = msg.data;
}


void ReinforcementLearning::algoritm(){

    // рандомное распределение весов
    Matrix weight_sgp = Matrix(6,6);
    double weight_in = (double)(rand())/RAND_MAX;
    double weight_gyro = (double)(rand())/RAND_MAX;
    weight_sgp.fill_random(1);

    // занулить главную диагональ
    weight_sgp.eye_zero();



    // инициализая спайков и потенциала
    Matrix voltage = Matrix(1,6);
    Matrix voltage_last = Matrix(1,6);
    Matrix spike_sgp = Matrix(1,6);
    Matrix spike_sgp_last = Matrix(1,6);
    double spike_in = 1.0;
    double spike_gyro = 0.0;
    spike_sgp.zero();
    spike_sgp_last.zero();
    voltage.zero();
    voltage_last.zero();
    spike_sgp_last.get_data()[0] = 1.0;
    spike_sgp_last.get_data()[2] = 1.0;
    spike_sgp_last.get_data()[4] = 1.0;


    std::cout <<"Random weight CPG" <<std::endl;
    weight_sgp.output();
    std::cout <<"Random weight in = "<< weight_in <<std::endl;
    std::cout <<"Random weight gyro = "<< weight_gyro <<std::endl;
    std::cout <<"Init spike GCP" <<std::endl;
    spike_sgp.output();
    spike_sgp_last.output();
    std::cout <<"Init spike in = " <<spike_in<<std::endl;
    std::cout <<"Init spike gyro = " <<spike_gyro<<std::endl;
    std::cout <<"Init voltage" <<std::endl;
    voltage.output();


    // Алгоритм
    // Сумма вклада всех соседних нейронов
    double sum_contribution_neighbours = 0.0;

    //ток
    double current_neuron;

    for( unsigned int time = 0; time < TIME_ITERATION; time ++){


        // 1) ПРОВЕРЯЕМ НА ВСПЫШКИ НЕЙРОНЫ
        for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++ ){
            for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON; neighbour++ ){
                // считает вклад в ток от соседних нейронов

                    sum_contribution_neighbours =  weight_sgp.get_data()[neighbour + NUMBER_NEURON * neuron] * spike_sgp_last.get_data()[ neighbour] + sum_contribution_neighbours;

            }

            // считаем ток
            current_neuron = weight_in * spike_in + weight_gyro * spike_gyro + sum_contribution_neighbours;
            // считаем потенциал
            voltage.get_data()[neuron] = voltage_last.get_data()[neuron]/LEAK_RATE + current_neuron;

            // если прошли порог
            spike_sgp.get_data()[neuron] = 0.0;
            if (voltage.get_data()[neuron] > VOLTAGE_TRESHOLD){
                std::cout <<"------------SPIKE NEURON = "<< neuron <<std::endl;
                spike_sgp.get_data()[neuron] = 1.0;
                voltage.get_data()[neuron] = 0.0;

            }
            // записываем в память текущее состояние потенциала нейрона
            voltage_last.get_data()[neuron] =  voltage.get_data()[neuron];
            sum_contribution_neighbours = 0.0;

        }

        // 2) ОБНОВЛЯЕМ ВЕСА
        ros::Duration(1.0).sleep();
        while (reward == reward_last){
            std::cout<< "Wait reward"<<std::endl;

        }
        reward_last = reward;
         for (unsigned int neuron = 0; neuron < NUMBER_NEURON * NUMBER_NEURON; neuron++ ){
            weight_sgp.get_data()[neuron] = weight_sgp.get_data()[neuron] + LEARNING_RATE * (double)(rand())/RAND_MAX * reward;
        }
        // занулить главную диагональ
        weight_sgp.eye_zero();

        std::cout <<" Weight in time =  "<< time <<std::endl;
        weight_sgp.output();
        std::cout <<" voltage in time =  "<< time <<std::endl;
        voltage.output();
        std::cout <<" spike_sgp in time =  "<< time <<std::endl;
        spike_sgp.output();



        if( time % ITERATION_SPIKE_IN == 0) spike_in = 0.0;



        for (unsigned int i = 0; i < spike_sgp.get_col() * spike_sgp.get_row(); i ++){
            spike_sgp_last.get_data()[i] = spike_sgp.get_data()[i];
        }

        std::cout <<" spike_sgp_last in time =  "<< time <<std::endl;
        spike_sgp_last.output();

    }
    std::cout <<"reward = " << reward <<std::endl;
}


