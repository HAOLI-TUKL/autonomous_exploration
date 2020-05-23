//
// Created by parallels on 2/5/20.
//



#include "../include/my_move_base/A_star.h"
#include <float.h>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>




void A_star::A_star_search() {
    cout.precision(8);
    openlist.clear();
    closelist.clear();
    path.clear();
    openlist.push_back(starting_point);
    int runningcount = 10000;
    while(runningcount!=0 ){
        runningcount -=1;
        sort(openlist.begin(),openlist.end(),[this](A_star::cell c1,A_star::cell c2){ return compare(c1,c2);});

        if(runningcount == 13000){cout<<" astar 13000 ...."<<endl;}
        if(runningcount == 10000){cout<<" astar 10000 ...."<<endl;}
        if(runningcount == 8000){cout<<" astar 8000 ...."<<endl;}
        if(runningcount == 5000){cout<<" astar 5000 ...."<<endl;}
        if(runningcount == 3000){cout<<" astar 3000 ...."<<endl;}
        if(runningcount == 2000){cout<<" astar 2000 ...."<<endl;}
        if(runningcount == 1000){cout<<" astar 1000 ...."<<endl;}
        if(runningcount == 800){cout<<" astar 800 ...."<<endl;}
        if(runningcount == 500){cout<<" astar 500 ...."<<endl;}
        if(runningcount == 300){cout<<" astar 300 ...."<<endl;}
        if(runningcount == 100){cout<<" astar 100 ...."<<endl;}
        A_star::cell focus = openlist[0];
        close_list_map_.data[focus.index] = 75;
        closelist.push_back(focus);
        if(openlist.size() == 0){
            cout<<" **can not find path to goal**"<<endl;
            search_success_ = false;
            break;
        }

        openlist.erase(openlist.begin());
        vector<A_star::cell> neig_vec;
        neig_vec = get_neighbor(focus);
        bool find_goal = false;
        for (int l = 0; l < neig_vec.size(); ++l) {
            if (get_distance_1(neig_vec[l].x,neig_vec[l].y,goal.x,goal.y) < 3){
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

}



void A_star::set_path_id() {
    A_star::cell c = closelist.back();
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
        A_star::cell parent_cell;

        for (int i = 0; i < closelist.size(); ++i) {
            if(closelist[i].id == parent_id){
                parent_cell = closelist[i];
            }

        }
        path.push_back(parent_cell);
        c = parent_cell;
    }

    cout<<"size of path before delete : "<<path.size()<<endl;
    if(path.size() < 4 ){ return;}
    int indexinpath = 0;
    for (vector<A_star::cell>::iterator it=path.begin(); it != path.end();) {
        if (indexinpath % 4 != 0){
            it = path.erase(it);
        } else{
            it++;
        }
        indexinpath++;
    }
    cout<<"size of path after delete : "<<path.size()<<endl;


}

bool A_star::compare(A_star::cell c1,A_star::cell c2){
    return (c1.f < c2.f);

};

double A_star::get_distance(A_star::cell c1,A_star::cell c2){
    double dis = sqrt(pow((c1.x - c2.x),2)+pow((c1.y - c2.y),2));
    return dis;
}
double A_star::get_distance_1(int x1,int y1,int x2,int y2){
    double dis = sqrt(pow((x1 - x2),2)+pow((y1 - y2),2));
    return dis;
}

vector<A_star::cell> A_star::get_neighbor(A_star::cell cell){
    previous_id_count+=1;
    vector<A_star::cell> neig_vec;

    id_for_use +=1;
    A_star::cell c1(cell.x - 1 , cell.y ,cell.index - 1 , cell.g +1,
                   get_distance_1(cell.x - 1, cell.y , goal.x , goal.y)  ,
                   cell.g +1+ get_distance_1(cell.x - 1 , cell.y , goal.x , goal.y), cell.index,cell.id,id_for_use);
    neig_vec.push_back(c1);//left



    id_for_use +=1;
     A_star::cell c2(cell.x , cell.y - 1,cell.index - my_map.info.width , cell.g +1,
                       get_distance_1( cell.x , cell.y - 1 , goal.x , goal.y),
                       cell.g +1+ get_distance_1(cell.x , cell.y - 1 , goal.x , goal.y) , cell.index,cell.id,id_for_use);
    neig_vec.push_back(c2);//down


    id_for_use +=1;
    A_star::cell c3(cell.x - 1, cell.y - 1,cell.index - my_map.info.width - 1 , cell.g +1 ,
                       get_distance_1(cell.x - 1, cell.y - 1 , goal.x , goal.y),
                       cell.g +1+ get_distance_1(cell.x - 1, cell.y - 1 , goal.x , goal.y),cell.index,cell.id,id_for_use);
    neig_vec.push_back(c3);// left down


    id_for_use +=1;
    A_star::cell c4(cell.x + 1 , cell.y,cell.index + 1 , cell.g +1 ,
                       get_distance_1(cell.x + 1 , cell.y , goal.x , goal.y) ,
                       cell.g +1+ get_distance_1(cell.x + 1, cell.y , goal.x , goal.y),cell.index,cell.id,id_for_use);
    neig_vec.push_back(c4);//right

    id_for_use +=1;
    A_star::cell c5(cell.x , cell.y + 1,cell.index + my_map.info.width , cell.g+1,
                       get_distance_1(cell.x , cell.y + 1 , goal.x , goal.y),
                       cell.g +1+ get_distance_1(cell.x , cell.y + 1 , goal.x , goal.y),cell.index,cell.id,id_for_use);
    neig_vec.push_back(c5);//up


    id_for_use +=1;
    A_star::cell c6(cell.x + 1, cell.y + 1,cell.index + my_map.info.width + 1,cell.g+1,
                       get_distance_1(cell.x + 1, cell.y + 1 , goal.x , goal.y),
                       cell.g +1+ get_distance_1(cell.x + 1, cell.y + 1 , goal.x , goal.y),cell.index,cell.id,id_for_use);
    neig_vec.push_back(c6);//right up


    id_for_use +=1;
    A_star::cell c7(cell.x + 1, cell.y - 1,cell.index - my_map.info.width + 1,cell.g+1,
                       get_distance_1(cell.x + 1, cell.y - 1,goal.x,goal.y),
                       cell.g +1+ get_distance_1(cell.x + 1, cell.y - 1 , goal.x , goal.y),cell.index,cell.id,id_for_use);
    neig_vec.push_back(c7);//right down


    id_for_use +=1;
    A_star::cell c8(cell.x - 1, cell.y + 1,cell.index + my_map.info.width - 1 , cell.g+1,
                       get_distance_1(cell.x - 1, cell.y + 1,goal.x,goal.y),
                       cell.g+1 + get_distance_1(cell.x - 1, cell.y + 1 , goal.x , goal.y),
                       cell.index,cell.id,id_for_use);
    neig_vec.push_back(c8);//left up


    int cou = 0;
    for (vector<A_star::cell>::iterator it=neig_vec.begin(); it != neig_vec.end();) {
        if (my_map.data[it->index] != 0){

            it = neig_vec.erase(it);
            cou+=1;

        } else{
            it++;
        }
    }


    return neig_vec;
}


vector<double > A_star::grid_to_coord(int* gridXY) {


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
std::vector<int>  A_star::coord_to_grid(std::vector<double> coord){
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

void A_star::SetStartingGoalPoint(double x_coord_s,double y_coord_s,double x_coord_g,double y_coord_g){
    std::vector<double> coord;
    coord.push_back(x_coord_s);
    coord.push_back(y_coord_s);
    std::vector<int> gridXY = coord_to_grid(coord);
    starting_point = *new A_star::cell() ;
    starting_point.x = gridXY[0];
    starting_point.y = gridXY[1];
    starting_point.index = gridXY[0] + gridXY[1]*my_map.info.width;
    starting_point.g = 0;

    goal = *new A_star::cell() ;
    std::vector<double> coord1;
    coord1.push_back(x_coord_g);
    coord1.push_back(y_coord_g);
    std::vector<int> gridXY1 = coord_to_grid(coord1);
    goal.x = gridXY1[0];
    goal.y = gridXY1[1];
    starting_point.h = get_distance(starting_point,goal);
    starting_point.f = starting_point.h + starting_point.g;
    goal.g = DBL_MAX;
    goal.h = 0;
    goal.f = goal.h+goal.g;
    goal.index = gridXY1[0] + gridXY1[1]*my_map.info.width;
}
