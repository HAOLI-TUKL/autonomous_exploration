//
// Created by parallels on 2/10/20.
//

#ifndef SRC_HYBRICASTAR_H
#define SRC_HYBRICASTAR_H

#include <vector>
#include <nav_msgs/OccupancyGrid.h>
using namespace std;

class HybricAStar{

public:
    class cell{
    public:
        cell(){};
        cell(int x,int y,int index):x(x),y(y),index(index){} ;
        cell(int x,int y,int index,double g, double h,double f,int parent_index):
                x(x),y(y),index(index),g(g),h(h),f(f),parent_index(parent_index){} ;

        cell(int x,int y,int index,double g, double h,double f,int parent_index,double x_coord,double y_coord,double theta,double vs,double steering_angle):
                x(x),y(y),index(index),g(g),h(h),f(f),parent_index(parent_index),x_coord(x_coord),y_coord(y_coord),theta(theta),vs(vs),steering_angle(steering_angle){} ;
        cell(int x,int y,int index,double g, double h,double f,int parent_index,double x_coord,double y_coord,double theta,double vs,double steering_angle,int parent_id,int id):
                x(x),y(y),index(index),g(g),h(h),f(f),parent_index(parent_index),x_coord(x_coord),y_coord(y_coord),theta(theta),vs(vs),steering_angle(steering_angle),parent_id(parent_id),id(id){} ;
        int x = 0; // grid index
        int y = 0; // grid index

        double x_coord = 0; // coord in map
        double y_coord = 0; // coord in map
        double theta = 0;//based on x axis, -pi to pi; direction of the car heading

        int index = 0;
        double g = 0;
        double f = 0;
        double h = 0 ;
        int parent_index = 0 ;
        double vs = 0;
        double steering_angle = 0;
        int id = 0;
        int parent_id = 0;

    };
    HybricAStar(){};
    ~HybricAStar(){};
    nav_msgs::OccupancyGrid my_map;
    nav_msgs::OccupancyGrid close_list_map_;//used for display the points in the close list
    nav_msgs::OccupancyGrid neighbor_map_;//used for display the points named neighbor

    vector<HybricAStar::cell> openlist;
    vector<HybricAStar::cell> closelist;
    HybricAStar::cell starting_point;
    HybricAStar::cell goal;
    vector<HybricAStar::cell> path;
    double delta_t = 0.4;//0.12 0.3N 0.6Y
    double length_of_car = 0.5075;
    vector<double> discrete_vs;// m/s
    vector<double> discrete_steering_angles;//radian alpha
    int previous_id_count = 0;
    int id_for_use = 0;
    bool search_success_ = true;
    void A_star_search();
    bool compare(HybricAStar::cell c1,HybricAStar::cell c2);
    double get_distance(HybricAStar::cell c1,HybricAStar::cell c2);
    double get_distance_1(int x1,int y1,int x2,int y2);
    vector<HybricAStar::cell> get_neighbor(HybricAStar::cell cell);
    vector<double > grid_to_coord(int* gridXY);
    std::vector<int> coord_to_grid(std::vector<double> coord);
    void initialize();
    void set_path_id();
    void SetStartingGoalPoint(double x_coord_s,double y_coord_s,double theta_s,double x_coord_g,double y_coord_g);

};


#endif //SRC_HYBRICASTAR_H
