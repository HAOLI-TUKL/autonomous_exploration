//
// Created by parallels on 1/30/20.
//

#include "../include/my_frontier_exploration/frontier_search.h"
#include <tgmath.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>

using namespace  std;


void frontier_search::ConfigInit(){
    const char ConfigFile[]= "./src/my_frontier_exploration/Configuration/config.txt";
    Config configSettings(ConfigFile);
    ratio_k_nearest_ = configSettings.Read("ratio_k_nearest_",2);
}

vector<frontier_search::cell> frontier_search::get_frontier_edge_cells(nav_msgs::OccupancyGrid map)
{

    cout<<"ratio_k_nearest_ : "<<ratio_k_nearest_<<endl;
    frontier_edge_cells.clear();
    frontier_regions.clear();
    frontiers.clear();
    frontiers_pool.clear();
    vector<frontier_search::cell> vector_free;
    for (int i = 0; i < map.data.size(); ++i)
    {
         if (map.data[i] == 0){ //free
             frontier_search::cell c((i+map.info.width)%map.info.width,floor(i/map.info.width),i);
             vector_free.push_back(c);
         }

    }
    cout<<"free cells : "<<vector_free.size()<<endl;
    vector<frontier_search::cell> edge_cells;
    for (int j = 0; j < vector_free.size(); ++j) {
        if (vector_free[j].x >= 1 && vector_free[j].y >= 1 && vector_free[j].x < map.info.width - 1 &&
            vector_free[j].y < map.info.height - 1)//cells not in the boundary of the map
        {

            bool have_unknown1 = (map.data[vector_free[j].index - 1] == -1) ? true : false;//check left
            bool have_unknown2 = (map.data[vector_free[j].index + 1] == -1) ? true : false;//check right
            bool have_unknown3 = (map.data[vector_free[j].index + map.info.width] == -1) ? true : false;//check up
            bool have_unknown4 = (map.data[vector_free[j].index - map.info.width] == -1) ? true : false;//check down
            bool have_unknown5 = (map.data[vector_free[j].index - map.info.width + 1] == -1) ? true : false;//check down right
            bool have_unknown6 = (map.data[vector_free[j].index - map.info.width - 1] == -1) ? true : false;//check down left
            bool have_unknown7 = (map.data[vector_free[j].index + map.info.width - 1] == -1) ? true : false;//check up left
            bool have_unknown8 = (map.data[vector_free[j].index + map.info.width + 1] == -1) ? true : false;//check up right
            if (have_unknown1 || have_unknown2 || have_unknown3 || have_unknown4 || have_unknown5 || have_unknown6 ||
                have_unknown7 || have_unknown8) {
                edge_cells.push_back(vector_free[j]);
            }
        } else{
            cout<<"############## reach boudary of map ############## "<<endl;
        }
    }

    if(edge_cells.size() == 0){cout<<"no more frontier_edge_cells can be found "<<endl;}
    return edge_cells;

}
std::vector<frontier_search::region> frontier_search::get_frontier_regions(std::vector<frontier_search::cell> frontier_edge_cells){

    std::cout<<"frontier_edge_cells : "<<frontier_edge_cells.size()<<std::endl;
    std::vector<frontier_search::region> vector;
    if(frontier_edge_cells.size() < ratio_k_nearest_){
        cout<<"there is no enough frontier_edge_cells for knearest "<<endl;
        return vector;
    }

    vector = frontier_search::knearest(frontier_edge_cells);

    return vector;
}



std::vector<frontier_search::region> frontier_search::knearest(std::vector<frontier_search::cell> frontier_edge_cells){

    cout<<"frontier_edge_cells size :"<<frontier_edge_cells.size()<<endl;
    int k_n = round(frontier_edge_cells.size()/ratio_k_nearest_);
    cout<<"k_n : "<<k_n<<endl;

    if(k_n <= 0){
        cout<<"no frontier_edge_cells is provided "<<endl;
    }
    int converge_count = 0;//if equal to k_n, means total center dont change any more
    vector<frontier_search::cell> centors_vec ;// initial centers
    vector<frontier_search::region> class_of_cells(k_n);
    for (int m = 0; m < k_n; ++m) {// initial
        frontier_search::region r;

        class_of_cells[m] = r;
    }
    for (int i = 0; i < k_n; ++i) {
        int max = frontier_edge_cells.size();
        int rand_index = rand()%max;
        centors_vec.push_back(frontier_edge_cells[rand_index]);
    }

    while (converge_count != class_of_cells.size()){
        for (int l = 0; l < class_of_cells.size(); ++l) {
            class_of_cells[l].vector.clear();
        }
        //for every cell
        for (int j = 0; j <  frontier_edge_cells.size(); ++j) {// for each cells in the edge
            vector<double> distance_vec ;
            for (int i = 0; i < k_n; ++i) {// get the distance from a cell to every center
                double distance = get_distance(frontier_edge_cells[j],centors_vec[i]);
                distance_vec.push_back(distance);
            }

            //which center index that this cell is closet to
            int smallest = min_element(distance_vec.begin(),distance_vec.end())-distance_vec.begin();
            class_of_cells[smallest].vector.push_back(frontier_edge_cells[j]) ;
            class_of_cells[smallest].centroid = centors_vec[smallest];

        }
        //center is no necessarily exactly located at one of the cells
        // delete a center if its corresponding class is empty
        int index_correponding_in_class = 0;
        for (vector<frontier_search::cell>::iterator it=centors_vec.begin(); it != centors_vec.end();) {
            if (class_of_cells[index_correponding_in_class].vector.size() ==0 ){
                cout<<"*************"<<" delete a center "<<"*************"<<endl;
                it = centors_vec.erase(it);
            } else{
                it++;
            }
            index_correponding_in_class++;
        }
        //delete a class if it is empty
        for (vector<frontier_search::region>::iterator it=class_of_cells.begin(); it != class_of_cells.end(); ) {
            if(it->vector.size() == 0){
                cout<<"*************"<<" delete a region without any cells "<<"*************"<<endl;
                it = class_of_cells.erase(it);
            } else{
                it++;
            }
        }
        converge_count = 0;
        for (int k = 0; k < class_of_cells.size(); k++) {
            frontier_search::cell new_center = update_center(class_of_cells[k]);
            if (new_center.index == centors_vec[k].index){
                converge_count += 1;
            }
            centors_vec[k] = new_center;
        }
    }

    return class_of_cells;

}

double frontier_search::get_distance(frontier_search::cell candidate,frontier_search::cell center){
    return std::sqrt(pow((candidate.x - center.x),2) + pow((candidate.y - center.y),2));
}
frontier_search::cell frontier_search::update_center(frontier_search::region one_class_of_cell){
    if(one_class_of_cell.vector.size() == 0){
        cout<<"*************"<<" input a region without any cells "<<"*************"<<endl;

    }
    double mean_x;
    double mean_y;
    double sum_x;
    double sum_y;
    for (int i = 0; i < one_class_of_cell.vector.size(); ++i) {
        sum_x += one_class_of_cell.vector[i].x ;
        sum_y += one_class_of_cell.vector[i].y ;
    }
    mean_x = sum_x / one_class_of_cell.vector.size();
    mean_y = sum_y / one_class_of_cell.vector.size();
    vector<double> distance_vec ;
    for (int j = 0; j < one_class_of_cell.vector.size(); ++j) {
      distance_vec.push_back( sqrt(pow((one_class_of_cell.vector[j].x - mean_x),2)+pow((one_class_of_cell.vector[j].y - mean_y),2)));
    }
    int smallest = min_element(distance_vec.begin(),distance_vec.end()) - distance_vec.begin();

    return one_class_of_cell.vector[smallest];


}
std::vector<frontier_search::frontier> frontier_search::get_frontiers(std::vector<frontier_search::region> frontier_regions){
    cout<<"before filter : "<<frontier_regions.size()<<endl;
    int thres = 0;
    std::vector<frontier_search::frontier> frontiers_vec;
    if(frontier_regions.size() == 0){
        return frontiers_vec;
    }
    for (int i = 0; i < frontier_regions.size(); ++i) {
        if (frontier_regions[i].vector.size() > thres){
            frontier_search::frontier frontier;
            frontier.vector = frontier_regions[i].vector;
            frontier.centroid = frontier_regions[i].centroid;
            frontiers_vec.push_back(frontier);
        }
    }
    cout<<"before filter : "<<frontiers_vec.size()<<endl;

    return frontiers_vec;
}
vector<double > frontier_search::grid_to_coord(int* gridXY) {

    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    double origin_z = map.info.origin.position.z;
    double q1 = map.info.origin.orientation.x;
    double q2 = map.info.origin.orientation.y;
    double q3 = map.info.origin.orientation.z;
    double q0 = map.info.origin.orientation.w;

    Eigen::MatrixXd H(4,4);
    H << 1 - 2*q2*q2 - 2*q3*q3 , 2*q1*q2 - 2*q0*q3 , 2*q1*q3 + 2*q0*q2 , origin_x,
             2*q1*q2 + 2*q0*q3 , 1 - 2*q1*q1 - 2*q3*q3 , 2*q2*q3 - 2*q0*q1 , origin_y,
             2*q1*q3 - 2*q0*q2 , 2*q2*q3 + 2*q0*q1 , 1 - 2*q1*q1 - 2*q2*q2 , origin_z,
             0,0,0,1;
    Eigen::VectorXd grid_xy(4,1);
    grid_xy << gridXY[0] * map.info.resolution,
               gridXY[1] * map.info.resolution,
               0 ,
               1;
    Eigen::VectorXd map_xy(4,1);
    map_xy = H * grid_xy ;
    vector<double > mapXY;
    mapXY.push_back(map_xy(0,0));
    mapXY.push_back(map_xy(1,0));

    return mapXY;

}
//check a block area around a frontier center, decide whether this block area has already detected in newest_stored_map
bool frontier_search::still_unknown(frontier_search::frontier frontier,inflation_obstacle_builder builder){
    int blockwidth = 5;
    int blockarea = (2*blockwidth) * (2*blockwidth);
    int thres = round(blockarea * 0.08);// 5
    int centerx = frontier.centroid.x;
    int centery = frontier.centroid.y;
    int stillunknowncount = 0;
    for(int j = -blockwidth; j < blockwidth;j++){// 10
        for(int k = -blockwidth; k < blockwidth;k++){// 10
            if(builder.newest_map.data[(centerx + j) + (centery + k) * builder.newest_map.info.width] == -1){
                stillunknowncount ++;
            }
        }
    }
    bool inside_obstacle = (builder.inflation_map.data[centerx + centery * builder.inflation_map.info.width] == 100) ||
            (builder.inflation_map.data[centerx + centery * builder.inflation_map.info.width] == 50);
    if(stillunknowncount <= thres || inside_obstacle == true){// surrounding is free or center is in the obstacle
        return false;
    }
    return true;
}

void frontier_search::build_dfs(std::vector<frontier_search::frontier> frontiers) {
    if (first_node_dsf_built_ == false) {
        dfs_.push_back(frontiers);
        first_node_dsf_built_ = true;
    } else {
        vector<bool> frontier_existed(frontiers.size(), false);
        for (int i = 0; i < dfs_.size(); i++) {
            for (int j = 0; j < dfs_[i].size(); j++) {
                for (int k = 0; k < frontiers.size(); k++) {
                    if (dfs_[i][j].centroid.index == frontiers[k].centroid.index) {
                        frontier_existed[k] = true;
                    }
                }

            }
        }
        vector<frontier_search::frontier> newnode;
        for (int i = 0; i < frontiers.size(); i++) {
            if (frontier_existed[i] == false) {
                newnode.push_back(frontiers[i]);
            }
        }
        if (newnode.size() != 0) {
            dfs_.push_back(newnode);
        }
    }
    return;
}

frontier_search::frontier frontier_search::get_frontier_based_on_dfs(inflation_obstacle_builder builder){
    cout<<"size of dfs_ : "<<dfs_.size()<<endl;
    cout<<"size of last element of dfs_ : "<<dfs_.back().size()<<endl;
    if(dfs_.size() == 0 ){
        cout<<"no frontier in dfs_ any more.. GoodBye"<<endl;
        exit(0);
    }
    vector<frontier_search::frontier>& lastnode = dfs_.back();
    cout<<"last node size : "<<lastnode.size()<<endl;
    vector<vector<double>> pos_ori;
    pos_ori = frontier_search::get_pos_ori("chassis");
    vector<double > pos;
    pos.push_back(pos_ori[0][0]);
    pos.push_back(pos_ori[0][1]);
    vector<int > gridXY = frontier_search::coord_to_grid(pos);
    int x = gridXY[0];
    int y = gridXY[1];
    vector<double> distance_vec ;
    for (int j = 0; j < lastnode.size(); ++j) {
        distance_vec.push_back( sqrt(pow((double(lastnode[j].centroid.x) - x),2)+pow((double(lastnode[j].centroid.y) - y),2)));
    }
    cout<<"size of distance : "<<distance_vec.size()<<endl;
    int smallest = min_element(distance_vec.begin(),distance_vec.end()) - distance_vec.begin();
    frontier_search::frontier f = lastnode[smallest];
    lastnode.erase(lastnode.begin()+smallest);
    cout<<"size of last element of dfs_ after delete: "<<dfs_.back().size()<<endl;
    if(dfs_.back().size() == 0 ){
        dfs_.pop_back();
    }
    if(still_unknown(f,builder) == false){// this frontier is no more unknown , find second closest frontier
        cout<<"not still_unknown : "<<f.centroid.x<<","<<f.centroid.y<<endl;
        f = get_frontier_based_on_dfs(builder);
    }
    return f;

}


frontier_search::frontier frontier_search::get_closest_frontier_direct(std::vector<frontier_search::frontier> frontiers,inflation_obstacle_builder builder){
    if(frontiers.size() == 0){
        cout<<"no frontier is generated any more; process finished; GoodBye !"<<endl;
        exit(0);
    }
    vector<vector<double>> pos_ori;
    pos_ori = frontier_search::get_pos_ori("chassis");
    vector<double > pos;
    pos.push_back(pos_ori[0][0]);
    pos.push_back(pos_ori[0][1]);
    vector<int > gridXY = frontier_search::coord_to_grid(pos);
    int x = gridXY[0];
    int y = gridXY[1];
    vector<double> distance_vec ;
    for (int j = 0; j < frontiers.size(); ++j) {
        distance_vec.push_back( sqrt(pow((double(frontiers[j].centroid.x) - x),2)+pow((double(frontiers[j].centroid.y) - y),2)));
    }

    int smallest = min_element(distance_vec.begin(),distance_vec.end()) - distance_vec.begin();

    frontier_search::frontier f = frontiers[smallest];
    frontiers.erase(frontiers.begin()+smallest);

    return f;
};

frontier_search::frontier frontier_search::get_closest_frontier(std::vector<frontier_search::frontier>* frontiers_pool,inflation_obstacle_builder builder){
    if(frontiers_pool->size() == 0){
        cout<<"no frontier in the frontiers_pool any more; process finished; GoodBye !"<<endl;
        exit(0);
    }
    vector<vector<double>> pos_ori;
    pos_ori = frontier_search::get_pos_ori("chassis");
    vector<double > pos;
    pos.push_back(pos_ori[0][0]);
    pos.push_back(pos_ori[0][1]);
    vector<int > gridXY = frontier_search::coord_to_grid(pos);
    int x = gridXY[0];
    int y = gridXY[1];


    vector<double> distance_vec ;
    for (int j = 0; j < frontiers_pool->size(); ++j) {
        distance_vec.push_back( sqrt(pow((double((*frontiers_pool)[j].centroid.x) - x),2)+pow((double((*frontiers_pool)[j].centroid.y) - y),2)));
    }

    int smallest = min_element(distance_vec.begin(),distance_vec.end()) - distance_vec.begin();

    frontier_search::frontier f = (*frontiers_pool)[smallest];

    frontiers_pool->erase(frontiers_pool->begin()+smallest);
    if(still_unknown(f,builder) == false){// this frontier is no more unknown , find second closest frontier
        cout<<"not still_unknown : "<<f.centroid.x<<","<<f.centroid.y<<endl;
        f = get_closest_frontier(frontiers_pool,builder);
    }

    return f;
}
std::vector<std::vector<double>> frontier_search::get_pos_ori(string focuslink){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("map", focuslink, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("map", focuslink,
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    vector<double> pos;
    pos.push_back(transform.getOrigin().x());
    pos.push_back(transform.getOrigin().y());
    pos.push_back(transform.getOrigin().z());
    vector<double> ori;
    ori.push_back(transform.getRotation().x());
    ori.push_back(transform.getRotation().y());
    ori.push_back(transform.getRotation().z());
    ori.push_back(transform.getRotation().w());
    vector<vector<double>> pos_ori;
    pos_ori.push_back(pos);
    pos_ori.push_back(ori);

    return pos_ori;


}

std::vector<int>  frontier_search::coord_to_grid(std::vector<double> coord){
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    double origin_z = map.info.origin.position.z;
    double q1 = map.info.origin.orientation.x;
    double q2 = map.info.origin.orientation.y;
    double q3 = map.info.origin.orientation.z;
    double q0 = map.info.origin.orientation.w;

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
    grid_xy = H.inverse() * map_xy / map.info.resolution;
    vector<int > gridXY;
    gridXY.push_back(int(grid_xy(0,0)));
    gridXY.push_back(int(grid_xy(1,0)));

    return gridXY;
}

