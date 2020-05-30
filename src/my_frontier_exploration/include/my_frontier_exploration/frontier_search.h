//
// Created by parallels on 1/30/20.
//

#ifndef SRC_FRONTIER_SEARCH_H
#define SRC_FRONTIER_SEARCH_H
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "inflation_obstacle_builder.h"
#include "../Config/Config.h"


class frontier_search {
public:
    class cell{
    public:
        cell(){};
        cell(int x,int y,int index):x(x),y(y),index(index){} ;
        int x;
        int y;
        int index;
    };
    class region{
    public:
        region(){};
        std::vector<frontier_search::cell> vector;
        frontier_search::cell centroid  ;
    };

    class frontier{
    public:
        frontier(){};
        std::vector<frontier_search::cell> vector;
        frontier_search::cell centroid  ;
    };

    frontier_search(){
        ConfigInit();
        first_node_dsf_built_ = false;
        previous_id_count = 0;
    };
    ~frontier_search(){};
    enum cell_state{unknown = -1,free = 0,occupied = 100};
    nav_msgs::OccupancyGrid map;
//    nav_msgs::OccupancyGrid newest_map_stored;

    std::vector<frontier_search::frontier> frontiers;// only store all the frontiers generated from a new-coming map
//    std::vector<frontier_search::frontier> frontiers_for_delete;//store to delete old markers;
    std::vector<frontier_search::frontier> frontiers_pool;//store all the frontiers from all the maps
    std::vector<std::vector<frontier_search::frontier>> dfs_;
    int previous_id_count ;
    std::vector<double> position;
    std::vector<double> orientation;
    frontier_search::frontier closet_frontier;
    std::vector<frontier_search::cell> frontier_edge_cells;
    std::vector<frontier_search::region> frontier_regions;

    std::vector<double>  grid_to_coord(int* coord);
    std::vector<frontier_search::cell> get_frontier_edge_cells(nav_msgs::OccupancyGrid map);
    std::vector<frontier_search::region> get_frontier_regions(std::vector<frontier_search::cell> frontier_edge_cells);
    std::vector<frontier_search::frontier> get_frontiers(std::vector<frontier_search::region> frontier_regions);
    std::vector<int>  coord_to_grid(std::vector<double> coord);
    //all the new frontiers would be put into frontiers_pool, then the closest one would be selected
    frontier_search::frontier get_closest_frontier(std::vector<frontier_search::frontier>* frontiers_pool,inflation_obstacle_builder builder);
    //the closest one among the new frontiers is selected, then rest of them are abandon;
    frontier_search::frontier get_closest_frontier_direct(std::vector<frontier_search::frontier> frontiers,inflation_obstacle_builder builder);
    //all the new frontiers are put into the dfs_tree, than closest one among the last node of the tree is selected
    frontier_search::frontier get_frontier_based_on_dfs(inflation_obstacle_builder builder);
    std::vector<std::vector<double>> get_pos_ori(string focuslink);
    void build_dfs(std::vector<frontier_search::frontier> frontiers);

private:
    bool first_node_dsf_built_ ;//used for indicating whether the first node of dsf has built
    int ratio_k_nearest_ ;//2 for small map
    bool still_unknown(frontier_search::frontier frontier,inflation_obstacle_builder builder);//decide whether a frontier is still unknown in newest_map_stored
    frontier_search::cell update_center(frontier_search::region  one_class_of_cell);
    std::vector<frontier_search::region> knearest(std::vector<frontier_search::cell> frontier_edge_cells);
    double get_distance(frontier_search::cell candidate,frontier_search::cell center);
    void ConfigInit();
};


#endif //SRC_FRONTIER_SEARCH_H
