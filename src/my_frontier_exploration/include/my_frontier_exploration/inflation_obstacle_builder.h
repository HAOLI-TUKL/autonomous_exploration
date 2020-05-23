//
// Created by parallels on 2/6/20.
//

#ifndef SRC_INFLATION_OBSTACLE_BUILDER_H
#define SRC_INFLATION_OBSTACLE_BUILDER_H



#include <nav_msgs/OccupancyGrid.h>
#include <vector>

using namespace std;
class inflation_obstacle_builder{
public:
    class cell{
    public:
        cell(){};
        cell(int x,int y,int index):x(x),y(y),index(index){} ;

        int x = 0;
        int y = 0;
        int index = 0;

    };
    nav_msgs::OccupancyGrid newest_map;
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid inflation_map;
    vector<inflation_obstacle_builder::cell> obstacle_edge_vec;
    vector<inflation_obstacle_builder::cell> obstacle_space;
    double inflation_scale = 6; // num of cells
    void set_obstacle_space();
    void set_obstacle_edge_vec();
    void inflation();




};




#endif //SRC_INFLATION_OBSTACLE_BUILDER_H
