//
// Created by parallels on 2/4/20.
//

#ifndef SRC_RTT_H
#define SRC_RTT_H

#include <vector>
#include "nav_msgs/OccupancyGrid.h"

using namespace std;


class RRT{
public:
    class cell{
    public:
        cell(){};
        cell(int x,int y,int index):x(x),y(y),index(index){} ;
        cell(int x,int y,int index,double x_coord , double y_coord ,int id,int parent_id):
        x(x),y(y),index(index),x_coord(x_coord),y_coord(y_coord),id(id),parent_id(parent_id){} ;
        int x = 0;
        int y = 0;
        int index = 0;
        double x_coord = 0;
        double y_coord = 0;
        int id = 0;
        int parent_id = 0;
    };

    RRT(){};
    ~RRT(){};
    RRT::cell starting_point;
    RRT::cell goal;
    nav_msgs::OccupancyGrid my_map;
    nav_msgs::OccupancyGrid new_map;
    vector<RRT::cell> path;
    bool search_success_ = false;
    void build_tree();
    void build_path();
    vector<double > grid_to_coord(int* gridXY);
    std::vector<int> coord_to_grid(std::vector<double> coord);
    void SetStartingGoalPoint(double x_coord_s,double y_coord_s,double x_coord_g,double y_coord_g);

private:
    vector<RRT::cell> tree;
    vector<int> index_free_space;
    double thres = 2;
    double step = 4;
    int id_for_alloc = 0;//id = 0 for the starting point,its parent is also 0 ; id = -1 for the goal
    void set_index_free_space();
    bool collisionChecking(RRT::cell near_c,RRT::cell new_c);
    vector<int> get_new_cell(vector<int> near_cell , vector<int> rand_cell );

};














#endif //SRC_RTT_H
