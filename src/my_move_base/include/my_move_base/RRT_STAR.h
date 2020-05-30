//
// Created by parallels on 5/28/20.
//

#ifndef MY_MOVE_BASE_RRT_STAR_H
#define MY_MOVE_BASE_RRT_STAR_H


#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "../Config/Config.h"
using namespace std;


class RRT_STAR{
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

    RRT_STAR(){
        circle_radius_ = 6;
        thres = 6;
        step = 4;
        id_for_alloc = 0;
        search_success_ = false;
    };
    ~RRT_STAR(){};
    RRT_STAR::cell starting_point;
    RRT_STAR::cell goal;
    nav_msgs::OccupancyGrid my_map;
    nav_msgs::OccupancyGrid new_map;
    vector<RRT_STAR::cell> tree;
    vector<int> index_free_space;
    vector<RRT_STAR::cell> path;
    bool search_success_ ;
    int id_for_alloc ;//id = 0 for the starting point,its parent is also 0 ; id = -1 for the goal
    void set_index_free_space();
    void build_tree();
    void build_path();
    vector<int> get_new_cell(vector<int> near_cell , vector<int> rand_cell );
    vector<double > grid_to_coord(int* gridXY);
    std::vector<int> coord_to_grid(std::vector<double> coord);
    void SetStartingGoalPoint(double x_coord_s,double y_coord_s,double x_coord_g,double y_coord_g);
private:

    double circle_radius_ ;
    double thres ;
    double step ;
    bool PathCollisionChecking(RRT_STAR::cell near_c,RRT_STAR::cell new_c);
    bool CellCollisionChecking(RRT_STAR::cell& candidate);
    double GetDistanceFromStartingPoint(RRT_STAR::cell& candiate);



};
#endif //MY_MOVE_BASE_RRT_STAR_H
