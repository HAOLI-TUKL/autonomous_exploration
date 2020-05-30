//
// Created by parallels on 2/5/20.
//

#ifndef SRC_A_STAR_H
#define SRC_A_STAR_H

#include <vector>
#include <nav_msgs/OccupancyGrid.h>
using namespace std;

class A_star{

public:
    class cell{
    public:
        cell(){};
        cell(int x,int y,int index):x(x),y(y),index(index){} ;
        cell(int x,int y,int index,double g, double h,double f,int parent_index):
                                 x(x),y(y),index(index),g(g),h(h),f(f),parent_index(parent_index){} ;

        cell(int x,int y,int index,double g, double h,double f,int parent_index,int parent_id,int id):
                x(x),y(y),index(index),g(g),h(h),f(f),parent_index(parent_index),parent_id(parent_id),id(id){} ;
        int x = 0;
        int y = 0;
        int index = 0;
        double g = 0;
        double f = 0;
        double h = 0 ;
        int parent_index = 0 ;

        int id = 0;
        int parent_id = 0;
    };
    nav_msgs::OccupancyGrid my_map;
    nav_msgs::OccupancyGrid close_list_map_;//used for display the points in the close list
    vector<A_star::cell> openlist;
    vector<A_star::cell> closelist;
    A_star::cell starting_point;
    A_star::cell goal;
    vector<A_star::cell> path;
    bool search_success_ = true;

    void A_star_search();
    vector<double > grid_to_coord(int* gridXY);
    std::vector<int> coord_to_grid(std::vector<double> coord);
    void SetStartingGoalPoint(double x_coord_s,double y_coord_s,double x_coord_g,double y_coord_g);

private:
    int id_for_use = 0;
    int previous_id_count = 0;
    bool compare(A_star::cell c1,A_star::cell c2);
    vector<A_star::cell> get_neighbor(A_star::cell cell);
    double get_distance(A_star::cell c1,A_star::cell c2);
    double get_distance_1(int x1,int y1,int x2,int y2);
    void set_path_id();

};




#endif //SRC_A_STAR_H
