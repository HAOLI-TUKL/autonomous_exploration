//
// Created by parallels on 2/10/20.
//

#include "../include/my_move_base/HybricAStar.h"
#include <float.h>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>


void HybricAStar::initialize() {

    discrete_vs.push_back(1);
    discrete_vs.push_back(-1);
//    discrete_vs.push_back(1.5);
//    discrete_vs.push_back(2);
//    discrete_vs.push_back(2.5);
//    discrete_vs.push_back(3);
//    discrete_vs.push_back(3.5);
//    discrete_vs.push_back(4);
//    discrete_vs.push_back(-1);
//    discrete_vs.push_back(-2);
//    discrete_vs.push_back(-3);
    discrete_steering_angles.push_back(1.04);//60
    discrete_steering_angles.push_back(0.92);//52
    discrete_steering_angles.push_back(0.785);//45
    discrete_steering_angles.push_back(0.52);//30
    discrete_steering_angles.push_back(0.4);//23
    discrete_steering_angles.push_back(0.26);//15
    discrete_steering_angles.push_back(0.17);//10
    discrete_steering_angles.push_back(0);
    discrete_steering_angles.push_back(-0.17);//-10
    discrete_steering_angles.push_back(-0.26);//-15
    discrete_steering_angles.push_back(-0.4);//-23
    discrete_steering_angles.push_back(-0.52);//-30
    discrete_steering_angles.push_back(-0.785);//-45
    discrete_steering_angles.push_back(-1.04);//-60

    close_list_map_ = my_map;
}



void HybricAStar::A_star_search() {
    cout.precision(8);
    int runningcount = 10000;
    //initize
    openlist.clear();
    closelist.clear();
    path.clear();

    openlist.push_back(starting_point);
    while(runningcount!=0 ){
        runningcount -=1;
        if(runningcount == 13000){cout<<"hybri astar 13000 ...."<<endl;}
        if(runningcount == 10000){cout<<"hybri astar 10000 ...."<<endl;}
        if(runningcount == 8000){cout<<"hybri astar 8000 ...."<<endl;}
        if(runningcount == 5000){cout<<"hybri astar 5000 ...."<<endl;}
        if(runningcount == 3000){cout<<"hybri astar 3000 ...."<<endl;}
        if(runningcount == 2000){cout<<"hybri astar 2000 ...."<<endl;}
        if(runningcount == 1000){cout<<"hybri astar 1000 ...."<<endl;}
        if(runningcount == 800){cout<<"hybri astar 800 ...."<<endl;}
        if(runningcount == 500){cout<<"hybri astar 500 ...."<<endl;}
        if(runningcount == 300){cout<<"hybri astar 300 ...."<<endl;}
        if(runningcount == 100){cout<<"hybri astar 100 ...."<<endl;}

        sort(openlist.begin(),openlist.end(),[this](HybricAStar::cell c1,HybricAStar::cell c2){ return compare(c1,c2);});


        HybricAStar::cell focus = openlist[0];
        close_list_map_.data[focus.index] = 75;
        closelist.push_back(focus);
        if(openlist.size() == 0){
            cout<<" **can not find path to goal**"<<endl;
            search_success_ = false;
            break;
        }

        openlist.erase(openlist.begin());
        vector<HybricAStar::cell> neig_vec;
        neig_vec = get_neighbor(closelist.back());
        bool find_goal = false;
        for (int l = 0; l < neig_vec.size(); ++l) {
            if(get_distance_1(neig_vec[l].x,neig_vec[l].y,goal.x,goal.y) < 3){//5
                cout<<" ** find path to goal**"<<endl;
                find_goal = true;
                search_success_ = true;
                break;
            }
        }
        if(find_goal == true){
            break;
        }
        vector<bool> exist_open_close;
        vector<int> cor ;
        for (int m = 0; m < neig_vec.size(); ++m) {
            exist_open_close.push_back(false);
        }
        for (int m1 = 0; m1 < neig_vec.size(); ++m1) {
            cor.push_back(-1);
        }

        for (int i = 0; i < neig_vec.size(); ++i) {// check existance, if exist, change f and g
            for (int j = 0; j < openlist.size(); ++j) {
                if(neig_vec[i].index == openlist[j].index){
                    exist_open_close[i] = true;
                    cor[i] = j;
                }
            }
        }
        for (int i = 0; i < neig_vec.size(); ++i) {// check existance, if exist, change f and g
            for (int j = 0; j < closelist.size(); ++j) {
                if(neig_vec[i].index == closelist[j].index ||neig_vec[i].index == focus.index){
                    exist_open_close[i] = true;
                }
            }
        }
        for (int n = 0; n < neig_vec.size(); ++n) {
            if(cor[n] != -1){
                if (openlist[cor[n]].f > neig_vec[n].f ){
                    openlist[cor[n]] = neig_vec[n];
                }
            }

        }

        for (int k = 0; k < neig_vec.size(); ++k) {
            if(exist_open_close[k] == false){//if no exist , add into openlist
                openlist.push_back(neig_vec[k]);

            }
        }


    }
    if(runningcount == 0){
        cout<<"running count exceeds;find no path!"<<endl;
        search_success_ = false;
    }
    if(search_success_ == false){ return;}
    set_path_id();

    for(int i=0;i<path.size();i++){
        cout<<i<<" x : "<<path[i].x_coord<<endl;
        cout<<i<<" y : "<<path[i].y_coord<<"\n"<<endl;
    }
    cout<<"path size : "<<path.size()<<endl;

}

void HybricAStar::set_path_id() {
    HybricAStar::cell c = closelist.back();
    path.push_back(goal);
    path.push_back(c);
    int count = 6000;
    while (true ){
        count -=1;
        if( c.id == starting_point.id){
            cout<<" break : "<<count<<endl;
            break;
        }
        int parent_id = c.parent_id;
        HybricAStar::cell parent_cell;

        for (int i = 0; i < closelist.size(); ++i) {
            if(closelist[i].id == parent_id){
                parent_cell = closelist[i];
            }

        }
        path.push_back(parent_cell);
        c = parent_cell;
    }

}

bool HybricAStar::compare(HybricAStar::cell c1,HybricAStar::cell c2){
    return (c1.f < c2.f);

};

double HybricAStar::get_distance(HybricAStar::cell c1,HybricAStar::cell c2){
    double dis = sqrt(pow((c1.x - c2.x),2)+pow((c1.y - c2.y),2));
    return dis;
}
double HybricAStar::get_distance_1(int x1,int y1,int x2,int y2){
    double dis = sqrt(pow((x1 - x2),2)+pow((y1 - y2),2));
    return dis;
}
vector<HybricAStar::cell> HybricAStar::get_neighbor(HybricAStar::cell cell){
    previous_id_count+=1;
    vector<HybricAStar::cell> neig_vec;
    for (int i = 0; i < discrete_vs.size(); ++i) {
        for (int j = 0; j < discrete_steering_angles.size(); ++j) {
            id_for_use+=1;
            double x_dot = discrete_vs[i] * cos(discrete_steering_angles[j]) * cos(cell.theta);
            double y_dot = discrete_vs[i] * cos(discrete_steering_angles[j]) * sin(cell.theta);
            double theta_dot = discrete_vs[i]/length_of_car * sin(discrete_steering_angles[j]);
            double x_new = cell.x_coord + x_dot*delta_t;
            double y_new = cell.y_coord + y_dot*delta_t;
            double theta_new = cell.theta + theta_dot*delta_t;


            double x_dot_1 = discrete_vs[i] * cos(discrete_steering_angles[j]) * cos(theta_new);
            double y_dot_1 = discrete_vs[i] * cos(discrete_steering_angles[j]) * sin(theta_new);
            theta_dot = discrete_vs[i]/length_of_car * sin(discrete_steering_angles[j]);
            x_new = x_new + x_dot_1*delta_t;
            y_new = y_new + y_dot_1*delta_t;
            theta_new = theta_new + theta_dot*delta_t;

            vector<double> coord;
            coord.push_back(x_new);
            coord.push_back(y_new);
            vector<int> grid_pos = coord_to_grid(coord);
            double h = get_distance_1(grid_pos[0],grid_pos[1],goal.x,goal.y);
            HybricAStar::cell c(grid_pos[0],grid_pos[1],grid_pos[0] + grid_pos[1]*my_map.info.width,
                                   cell.g+1, h ,cell.g+1+h,cell.index,coord[0],coord[1],theta_new,
                                   discrete_vs[i],discrete_steering_angles[j],cell.id,id_for_use);
            neig_vec.push_back(c);

        }
    }
    int cou = 0;
    for (vector<HybricAStar::cell>::iterator it=neig_vec.begin(); it != neig_vec.end();) {
        if (my_map.data[it->index] != 0){

            it = neig_vec.erase(it);
            cou+=1;

        } else{
            it++;
        }
    }


    return neig_vec;
}

vector<double > HybricAStar::grid_to_coord(int* gridXY) {


    double origin_x = my_map.info.origin.position.x;
    double origin_y = my_map.info.origin.position.y;
    double origin_z = my_map.info.origin.position.z;
    double q1 = my_map.info.origin.orientation.x;
    double q2 = my_map.info.origin.orientation.y;
    double q3 = my_map.info.origin.orientation.z;
    double q0 = my_map.info.origin.orientation.w;

    Eigen::MatrixXd H(4,4);
    H << 1 - 2*q2*q2 - 2*q3*q3 , 2*q1*q2 - 2*q0*q3 , 2*q1*q3 + 2*q0*q2 , origin_x,
            2*q1*q2 + 2*q0*q3 , 1 - 2*q1*q1 - 2*q3*q3 , 2*q2*q3 - 2*q0*q1 , origin_y,
            2*q1*q3 - 2*q0*q2 , 2*q2*q3 + 2*q0*q1 , 1 - 2*q1*q1 - 2*q2*q2 , origin_z,
            0,0,0,1;
    Eigen::VectorXd grid_xy(4,1);
    grid_xy << gridXY[0] * my_map.info.resolution,
            gridXY[1] * my_map.info.resolution,
            0 ,
            1;
    Eigen::VectorXd map_xy(4,1);
    map_xy = H * grid_xy ;
    vector<double > mapXY;
    mapXY.push_back(map_xy(0,0));
    mapXY.push_back(map_xy(1,0));


    return mapXY;

}
std::vector<int>  HybricAStar::coord_to_grid(std::vector<double> coord){
    double origin_x = my_map.info.origin.position.x;
    double origin_y = my_map.info.origin.position.y;
    double origin_z = my_map.info.origin.position.z;
    double q1 = my_map.info.origin.orientation.x;
    double q2 = my_map.info.origin.orientation.y;
    double q3 = my_map.info.origin.orientation.z;
    double q0 = my_map.info.origin.orientation.w;

    Eigen::MatrixXd H(4,4);
    H << 1 - 2*q2*q2 - 2*q3*q3 , 2*q1*q2 - 2*q0*q3 , 2*q1*q3 + 2*q0*q2 , origin_x,
            2*q1*q2 + 2*q0*q3 , 1 - 2*q1*q1 - 2*q3*q3 , 2*q2*q3 - 2*q0*q1 , origin_y,
            2*q1*q3 - 2*q0*q2 , 2*q2*q3 + 2*q0*q1 , 1 - 2*q1*q1 - 2*q2*q2 , origin_z,
            0,0,0,1;
    Eigen::VectorXd map_xy(4,1);
    map_xy << coord[0] ,
            coord[1] ,
            0 ,
            1;
    Eigen::VectorXd grid_xy(4,1);
    grid_xy = H.inverse() * map_xy / my_map.info.resolution;

    vector<int > gridXY;
    gridXY.push_back(int(grid_xy(0,0)));
    gridXY.push_back(int(grid_xy(1,0)));

    return gridXY;
}

void HybricAStar::SetStartingGoalPoint(double x_coord_s,double y_coord_s,double theta_s,double x_coord_g,double y_coord_g){

    starting_point.g = 0;
    std::vector<double> coord;
    coord.push_back(x_coord_s);
    coord.push_back(y_coord_s);
    std::vector<int> gridXY = coord_to_grid(coord);

    starting_point.x_coord = x_coord_s;
    starting_point.y_coord = y_coord_s;
    starting_point.x = gridXY[0];
    starting_point.y = gridXY[1];
    starting_point.index = gridXY[0] + gridXY[1]*my_map.info.width;

    starting_point.theta = theta_s;
    starting_point.vs = 0;
    starting_point.steering_angle = 0;

    goal.x_coord = x_coord_g;
    goal.y_coord = y_coord_g;
    std::vector<double> coord1;
    coord1.push_back(x_coord_g);
    coord1.push_back(y_coord_g);
    std::vector<int> gridXY1 = coord_to_grid(coord1);
    goal.x = gridXY1[0];
    goal.y = gridXY1[1];
    goal.index = gridXY1[0] + gridXY1[1]*my_map.info.width;

    starting_point.h = get_distance(starting_point,goal);
    starting_point.f = starting_point.h+starting_point.g;

    goal.g = DBL_MAX;
    goal.h = 0;

    goal.theta = 0;
    goal.vs = 0;
    goal.steering_angle = 0;



}