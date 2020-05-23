//
// Created by parallels on 4/18/20.
//

#ifndef MY_MOVE_BASE_PID_H
#define MY_MOVE_BASE_PID_H
#include "HybricAStar.h"

using namespace std;

class Pid{
public:
    Pid(){};
    ~Pid(){};

    vector<HybricAStar::cell> path_;
    double p_vel_ = 0.45;//not to be large, which causes overshoot 0.3
    double p_rot_vel_ = 1;//select to make output same as the angle difference 1
    double i_vel_ = 0.05; //0.04
    double pos_threshold_ = 0.3;
    double sum_error_ = 0;
    int count_keep_rotating_ = 0;//record the counts of keeping rotating, which reflect whether the car gets stuck;
    vector<double> SetVel(vector<vector<double>> baselinkposori,vector<vector<double>> frontsteeringposori,vector<double> goal_pos);


};









#endif //MY_MOVE_BASE_PID_H
