//
// Created by parallels on 2/4/20.
//

#include "../include/my_move_base/RTT.h"
#include <stdlib.h>
#include <tgmath.h>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <Eigen/Dense>
void RRT::set_index_free_space(){
    vector<int> index_free;
    for (int i = 0; i < my_map.data.size(); ++i)
    {
        if (my_map.data[i] == 0){
            index_free.push_back(i);
        }

    }
    index_free_space = index_free;
    return ;

}

void RRT::build_tree() {
    new_map = my_map;
    set_index_free_space();
    tree.clear();
    path.clear();
    search_success_ = false;
    tree.push_back(starting_point);
    double count_skip;
    int max = index_free_space.size();
    int runningcount = 30000;
    while (runningcount != 0 ) {
        runningcount --;
        srand(runningcount);
        int rand_index = rand()%max;
        int rand_x = (index_free_space[rand_index]+ my_map.info.width)%my_map.info.width;
        int rand_y = floor(index_free_space[rand_index]/my_map.info.width);
        new_map.data[rand_x+rand_y*my_map.info.width] = 50;

        vector<double> distance_vec;
        for (int j = 0; j < tree.size(); ++j) {

            distance_vec.push_back(sqrt(pow((tree[j].x - rand_x),2)+pow((tree[j].y - rand_y),2)));
        }

        auto smallest = min_element(distance_vec.begin(),distance_vec.end()) - distance_vec.begin();
        RRT::cell near_cell = tree[smallest];


        vector<int> rand_cell;
        rand_cell.push_back(rand_x);
        rand_cell.push_back(rand_y);
        vector<int> near;
        near.push_back(near_cell.x);
        near.push_back(near_cell.y);

        vector<int> new_ce =  RRT::get_new_cell( near ,  rand_cell  );

        int gridXY[2] = {new_ce[0],new_ce[2]};
        vector<double> coordXY = grid_to_coord(gridXY);
        RRT::cell n_c(new_ce[0],new_ce[1],new_ce[0]+new_ce[1]*my_map.info.width,coordXY[0],coordXY[1],id_for_alloc,0);
        id_for_alloc++;
        new_map.data[new_ce[0]+new_ce[1]*my_map.info.width] = 80;

        if(collisionChecking(near_cell,n_c)){

            n_c.parent_id = near_cell.id;
            tree.push_back(n_c);


        }
        else{
            cout<<"collision !!"<<endl;
            continue;
        }
        if (sqrt((goal.x-n_c.x)*(goal.x-n_c.x) + (goal.y-n_c.y)*(goal.y-n_c.y)) < thres){
            cout<<"**********reach goal*********"<<endl;
            search_success_ = true;
            break;
        }

    }
    if(runningcount == 0 && search_success_ == false){
        cout<<"**********no path found*********"<<endl;
        return;
    } else{
        build_path();
        return;
    }



}
bool RRT::collisionChecking(RRT::cell near_c,RRT::cell new_c){
    int delta_x;
    if (new_c.x > near_c.x){
        delta_x = 1;
        int count = (new_c.x - near_c.x) / delta_x;
        for (int i = 1; i < count+1; ++i) {
            double k = (new_c.y - near_c.y)/ (new_c.x - near_c.x) ;
            if(my_map.data[(near_c.x+delta_x*i) + (near_c.y + floor(k * delta_x *i)) * my_map.info.width] != 0){
                return false;
            };
        }
    } else if (new_c.x < near_c.x){
        delta_x = -1;
        int count = (new_c.x - near_c.x) / delta_x;
        for (int i = 1; i < count+1; ++i) {
            double k = (new_c.y - near_c.y)/ (new_c.x - near_c.x) ;
            if(my_map.data[(near_c.x+delta_x*i) + (near_c.y + floor(k * delta_x *i)) * my_map.info.width] != 0){
                return false;
            };
        }
    } else{
        if(new_c.y >= near_c.y){
            int count = new_c.y - near_c.y;
            for (int i = 1; i < count+1; ++i) {
                if(my_map.data[near_c.x + (near_c.y + i)* my_map.info.width] != 0){
                    return false;
                }
            }
        } else{
            int count = near_c.y - new_c.y;
            for (int i = 1; i < count+1; ++i) {
                if(my_map.data[near_c.x + (near_c.y - i)* my_map.info.width] != 0){
                    return false;
                }
            }
        }
    }
    return true;
}
void RRT::build_path(){
    if(search_success_ == false){ return;}
    path.push_back(goal);
    RRT::cell c = tree.back();
    int count = 10000;
    while (count != 0 ){
        count -=1;
        if( c.id == starting_point.id){
            cout<<" break : "<<count<<endl;
            break;
        }
        int parent_id = c.parent_id;
        RRT::cell parent_cell;

        for (int i = 0; i < tree.size(); ++i) {
            if(tree[i].id == parent_id){
                parent_cell = tree[i];
            }

        }
        path.push_back(parent_cell);
        c = parent_cell;
    }

};

vector<int> RRT::get_new_cell(vector<int> near_cell , vector<int> rand_cell  ){
    vector<int> circle_candidate;
    for (int i = 0; i < index_free_space.size(); ++i) {
        int x = (index_free_space[i]+ my_map.info.width)%my_map.info.width;
        int y = floor(index_free_space[i]/my_map.info.width);
        double dis_to_center = sqrt(pow((x - near_cell[0]),2) + pow((y - near_cell[1]),2));
        if(  abs(dis_to_center - step) < 0.5){
            circle_candidate.push_back(index_free_space[i]);
        }
    }
    vector<double> distance_vec;
    for (int j = 0; j < circle_candidate.size(); ++j) {
        int x = (circle_candidate[j]+ my_map.info.width)%my_map.info.width;
        int y = floor(circle_candidate[j]/my_map.info.width);

        distance_vec.push_back(sqrt(pow((x - rand_cell[0]),2)+pow((y - rand_cell[1]),2)));
    }
    int smallest = min_element(distance_vec.begin(),distance_vec.end()) - distance_vec.begin();
    int new_index = circle_candidate[smallest];
    vector<int> new_cell;
    new_cell.push_back((new_index+ my_map.info.width)%my_map.info.width);
    new_cell.push_back(floor(new_index/my_map.info.width));
    return new_cell;
}

vector<double > RRT::grid_to_coord(int* gridXY) {
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
std::vector<int>  RRT::coord_to_grid(std::vector<double> coord){
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
void RRT::SetStartingGoalPoint(double x_coord_s,double y_coord_s,double x_coord_g,double y_coord_g){
    std::vector<double> coord;
    coord.push_back(x_coord_s);
    coord.push_back(y_coord_s);
    std::vector<int> gridXY = coord_to_grid(coord);
    starting_point = *new RRT::cell() ;
    starting_point.x = gridXY[0];
    starting_point.y = gridXY[1];
    starting_point.index = gridXY[0] + gridXY[1]*my_map.info.width;

    goal = *new RRT::cell() ;
    std::vector<double> coord1;
    coord1.push_back(x_coord_g);
    coord1.push_back(y_coord_g);
    std::vector<int> gridXY1 = coord_to_grid(coord1);
    goal.x = gridXY1[0];
    goal.y = gridXY1[1];
    goal.index = gridXY1[0] + gridXY1[1]*my_map.info.width;
};
