//
// Created by parallels on 4/18/20.
//

#ifndef MY_MOVE_BASE_PID_H
#define MY_MOVE_BASE_PID_H
#include "HybricAStar.h"
#include "../Config/Config.h"
using namespace std;

class Pid{
public:
    Pid(){
        p_vel_ = 0.45;//not to be large, which causes overshoot 0.3
        p_rot_vel_ = 1;//select to make output same as the angle difference 1
        i_vel_ = 0.05; //0.04
        pos_threshold_ = 0.3;
        sum_error_ = 0;
        count_keep_rotating_ = 0;//record the counts of keeping rotating, which reflect whether the car gets stuck;
        ConfigInit();
    };
    ~Pid(){};
    vector<HybricAStar::cell> path_;
    vector<double> SetVel(vector<vector<double>> baselinkposori,vector<vector<double>> frontsteeringposori,vector<double> goal_pos);

private:
    double p_vel_ ;//not to be large, which causes overshoot 0.3
    double p_rot_vel_;//select to make output same as the angle difference 1
    double i_vel_ ; //0.04
    double pos_threshold_ ;
    double sum_error_ ;
    int count_keep_rotating_ ;//record the counts of keeping rotating, which reflect whether the car gets stuck;
    double upper_limit_;
    void ConfigInit();
};









#endif //MY_MOVE_BASE_PID_H
