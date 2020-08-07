//
// Created by parallels on 5/28/20.
//

#include "../include/my_move_base/RRT_STAR.h"
#include <stdlib.h>
#include <tgmath.h>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <Eigen/Dense>
#include <time.h>
void RRT_STAR::set_index_free_space(){
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

void RRT_STAR::build_tree() {
//    cout<<"11 "<<endl;

    new_map = my_map;
    set_index_free_space();
    tree.clear();
    path.clear();
    search_success_ = false;
    tree.push_back(starting_point);
    double count_skip;
    int max = index_free_space.size();
//    cout<<"size of ..... "<<index_free_space.size()<<endl;
    const char ConfigFile[]= "./src/my_move_base/Configuration/config.txt";
    Config configSettings(ConfigFile);
    int runningcount = 3000;//configSettings.Read("optimization_count",3000);
//    int runningcount = 3000;
//    cout<<"22 "<<endl;
    double duration = 0 ;
    clock_t t1 = clock();
    while (runningcount != 0 ) {

//        cout<<" running count : "<<runningcount<<" ; duration : "<<duration<<endl;

        runningcount --;
        srand(runningcount);
//        cout<<"221 "<<endl;

        int rand_index = rand()%max;

//        cout<<"22 2"<<endl;
        int rand_x = (index_free_space[rand_index]+ my_map.info.width)%my_map.info.width;
//        cout<<"22 3"<<endl;
        int rand_y = floor(index_free_space[rand_index]/my_map.info.width);
        //new_map.data[rand_x+rand_y*my_map.info.width] = 50;
//        cout<<"33 "<<endl;
        vector<double> distance_vec;
        for (int j = 0; j < tree.size(); ++j) {

            distance_vec.push_back(sqrt(pow((tree[j].x - rand_x),2)+pow((tree[j].y - rand_y),2)));
        }

        auto smallest = min_element(distance_vec.begin(),distance_vec.end()) - distance_vec.begin();
        RRT_STAR::cell near_cell = tree[smallest];
//        cout<<"44 "<<endl;

        vector<int> rand_cell;
        rand_cell.push_back(rand_x);
        rand_cell.push_back(rand_y);
        vector<int> near;
        near.push_back(near_cell.x);
        near.push_back(near_cell.y);

        vector<int> new_ce =  RRT_STAR::get_new_cell( near ,  rand_cell  );
//        cout<<"55 "<<endl;
        int gridXY[2] = {new_ce[0],new_ce[2]};
        vector<double> coordXY = grid_to_coord(gridXY);
        RRT_STAR::cell n_c(new_ce[0],new_ce[1],new_ce[0]+new_ce[1]*my_map.info.width,coordXY[0],coordXY[1],id_for_alloc,0);
        id_for_alloc++;

//        cout<<"1 "<<endl;

        if(CellCollisionChecking(n_c)){

            vector<double> dis_candidate_to_n_c;//collect the distances from the candidates to the n_c
            vector<int> inside_circle_cell_index;//collect all the indice of the cells insider the circle
            for(int index = 0; index < tree.size(); index++ ){
                double dis = sqrt(pow((tree[index].x - n_c.x),2)+pow((tree[index].y - n_c.y),2));
                if (dis <= circle_radius_){
                    dis_candidate_to_n_c.push_back(dis);
                    inside_circle_cell_index.push_back(index);
                }
            }
//            cout<<"size of dis_candidate_to_n_c : "<<dis_candidate_to_n_c.size()<<endl;
//            cout<<"size of inside_circle_cell_index : "<<inside_circle_cell_index.size()<<endl;

//            cout<<"2 "<<endl;

            vector<double> dis_startingcell_to_candidate;//collect the distances from starting point to the candidates in the circle
            for(auto& index : inside_circle_cell_index){
                double distance = GetDistanceFromStartingPoint(tree[index]);
//                cout<<"dis from starting to candidates : "<<distance<<endl;
                dis_startingcell_to_candidate.push_back(distance);
            }
            vector<double> dis_startingcell_to_n_c;//collect the distances from starting point to the n_c
            for(int i = 0;i < dis_startingcell_to_candidate.size();i++){
                dis_startingcell_to_n_c.push_back(dis_candidate_to_n_c[i] + dis_startingcell_to_candidate[i]);
            }
            vector<double> dis_tmp = dis_startingcell_to_n_c;//used for the case when collision happens
            bool cell_has_added = false;
            int parent_index;
            while (std::count(dis_tmp.begin(), dis_tmp.end(), 10000.0) != dis_tmp.size() ){
//                cout<<"count 10000 : "<<std::count(dis_tmp.begin(), dis_tmp.end(), 10000.0) <<endl;
//                cout<<"count dis-tmp  : "<<dis_tmp.size() <<endl;
                auto smallest = min_element(dis_tmp.begin(),dis_tmp.end()) - dis_tmp.begin();
                if(PathCollisionChecking(tree[inside_circle_cell_index[smallest]],n_c)){
                    parent_index = smallest;
                    n_c.parent_id = tree[inside_circle_cell_index[smallest]].id;
                    tree.push_back(n_c);
//                    new_map.data[new_ce[0]+new_ce[1]*my_map.info.width] = 80;
                    cell_has_added = true;
                    break;
                } else{
                    dis_tmp[smallest] = 10000.0;
//                    cout<<"no parent can be reached. "<<endl;
                }
            }
//            cout<<"3 "<<endl;


            if(cell_has_added == false){ continue;}
            for(int i = 0 ;  i < inside_circle_cell_index.size();i++ ){
                if( i != parent_index){
                    double dis_starting_nc_candidate = dis_startingcell_to_candidate[parent_index] + dis_candidate_to_n_c[parent_index] + dis_candidate_to_n_c[i];
                    if(dis_startingcell_to_candidate[i] > dis_starting_nc_candidate){
                        if(PathCollisionChecking(tree[inside_circle_cell_index[i]],n_c)){
                            tree[inside_circle_cell_index[i]].parent_id = n_c.id;
//                            cout<<" rewiring ... "<<endl;

                        } else{
//                            cout<<"collision happens while rewiring ... "<<endl;
                        }
                    }
                } else{
                }
            }

//            cout<<"4 "<<endl;

        }


        if (sqrt((goal.x-n_c.x)*(goal.x-n_c.x) + (goal.y-n_c.y)*(goal.y-n_c.y)) < thres){
            cout<<"**********reach goal*********"<<endl;
            search_success_ = true;
        }

    }
//    if( search_success_ == false){
//        cout<<"**********no path found*********"<<endl;
//        return;
//    } else{
//        build_path();
//        return;
//    }

     build_path();
     duration = (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000;
     cout<<"duration : "<<duration<<endl;
}
bool RRT_STAR::CellCollisionChecking(RRT_STAR::cell& candidate){
    return my_map.data[candidate.x + candidate.y  * my_map.info.width] == 0;
};

double RRT_STAR::GetDistanceFromStartingPoint(RRT_STAR::cell& candiate){
    RRT_STAR::cell* search_pointer;
    double distance_from_candidate_to_starting_point = 0;
    for(auto& cell : tree){
        if(candiate.id == cell.id){
            search_pointer = &cell;
//            cout<<"set!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        }
    }
    int count = 10000;
    while (count != 0 ){
        count -=1;
        if( search_pointer->id == starting_point.id){
            break;
        }
        int parent_id = search_pointer->parent_id;
//        cout<<"parent_id : "<<parent_id<<endl;
        for (int i = 0; i < tree.size(); ++i) {
            if(tree[i].id == parent_id){
                double dis = sqrt(pow((tree[i].x - search_pointer->x),2)+pow((tree[i].y - search_pointer->y),2));
                distance_from_candidate_to_starting_point += dis;
                search_pointer = &tree[i];
//                cout<<"find parent"<<endl;
                break;
            }
        }
    }
    return distance_from_candidate_to_starting_point;
}


bool RRT_STAR::PathCollisionChecking(RRT_STAR::cell near_c,RRT_STAR::cell new_c){
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
void RRT_STAR::build_path(){
//    if(search_success_ == false){ return;}
//    for(auto& cell : tree){
//        if(cell.id == goal_id_after_search_){
//            path.push_back(cell);
//        }
//    }

    double min_dis = 10000;
    int cloest_to_goal_index = 0;
    for(int i = 0 ; i<tree.size();i++){
        double new_dis = sqrt(pow((tree[i].x - goal.x),2)+pow((tree[i].y - goal.y),2));
        if( new_dis < min_dis){
            min_dis = new_dis;
            cloest_to_goal_index = i;
        }
    }
    path.push_back(tree[cloest_to_goal_index]);
//    path.push_back(goal);
    RRT_STAR::cell c = path.back();
    int count = 10000;
    while (count != 0 ){
        count -=1;
        if( c.id == starting_point.id){
            cout<<" break : "<<count<<endl;
            break;
        }
        int parent_id = c.parent_id;
        RRT_STAR::cell parent_cell;

        for (int i = 0; i < tree.size(); ++i) {
            if(tree[i].id == parent_id){
                parent_cell = tree[i];
            }

        }
        path.push_back(parent_cell);
        c = parent_cell;
    }

};

vector<int> RRT_STAR::get_new_cell(vector<int> near_cell , vector<int> rand_cell  ){
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

vector<double > RRT_STAR::grid_to_coord(int* gridXY) {
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
std::vector<int>  RRT_STAR::coord_to_grid(std::vector<double> coord){
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
void RRT_STAR::SetStartingGoalPoint(double x_coord_s,double y_coord_s,double x_coord_g,double y_coord_g){
    std::vector<double> coord;
    coord.push_back(x_coord_s);
    coord.push_back(y_coord_s);
    std::vector<int> gridXY = coord_to_grid(coord);
    starting_point = *new RRT_STAR::cell() ;
    starting_point.x = gridXY[0];
    starting_point.y = gridXY[1];
    starting_point.index = gridXY[0] + gridXY[1]*my_map.info.width;

    goal = *new RRT_STAR::cell() ;
    std::vector<double> coord1;
    coord1.push_back(x_coord_g);
    coord1.push_back(y_coord_g);
    std::vector<int> gridXY1 = coord_to_grid(coord1);
    goal.x = gridXY1[0];
    goal.y = gridXY1[1];
    goal.index = gridXY1[0] + gridXY1[1]*my_map.info.width;
};
