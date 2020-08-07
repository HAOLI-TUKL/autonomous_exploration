//
// Created by parallels on 2/4/20.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "../include/my_move_base/RRT.h"
#include <stdlib.h>
#include <time.h>
#include "../include/my_move_base/A_star.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "../include/my_move_base/inflation_obstacle_builder.h"
#include "../include/my_move_base/HybricAStar.h"
#include "../include/my_move_base/pid.h"
#include <Eigen/Dense>

HybricAStar Hastar;

ros::Publisher markerArray_pub;
ros::Publisher marker_pub;
inflation_obstacle_builder builder;
ros::Publisher pub_inf;
ros::Publisher pub_test;

std::vector<double> markerarrayx;// for display
std::vector<double> markerarrayy;//for display
bool receive_starting_from_rviz = false;
bool receive_goal_from_rviz = false;
int type_from_rviz = 0 ;
  double tmpx = 0;
  double tmpy = 0;



void Initmarker(HybricAStar::cell cell ,int id){
    int gridXY[2] = {cell.x , cell.y};
//    vector<int> gridXY;
//    gridXY.push_back(cell.x);
//    gridXY.push_back(cell.y);
//    std::cout<<" closest x in grid :"<<gridXY[0]<<std::endl;
//    std::cout<<" closest y in grid :"<<gridXY[1]<<std::endl;
    std::vector<double > mapXY;
    mapXY = Hastar.grid_to_coord(gridXY);
//    std::cout<<" closest x :"<<mapXY[0]<<std::endl;
//    std::cout<<" closest y :"<<mapXY[1]<<std::endl;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
//    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mapXY[0];
    marker.pose.position.y = mapXY[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = -0.7071;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.7071;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f * id/1000;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
//    std::cout<<"****** pub marker *******"<<std::endl;
}
void InitPubMarkerarray(vector<HybricAStar::cell> path ){
    // clear all the marker
    visualization_msgs::MarkerArray ma;
    for(int i =0 ;i< markerarrayx.size();i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.pose.position.x = markerarrayx[i];
        marker.pose.position.y = markerarrayy[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = -0.7071;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.7071;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        ma.markers.push_back(marker);
    }
    markerArray_pub.publish(ma);
    ma.markers.clear();
    markerarrayx.clear();
    markerarrayy.clear();

    if(path.size() == 0){
        std::cout<<"no path"<<std::endl;
    }else{
        visualization_msgs::MarkerArray ma;
        for (int j = 0; j < path.size(); ++j) {
            int  gridXY[2] = {int((path.at(j).index + Hastar.my_map.info.width)%Hastar.my_map.info.width),
                              int(floor(path.at(j).index/Hastar.my_map.info.width))};
//            vector<int> gridXY;
//            gridXY.push_back(int((path.at(j).index + map->info.width)%map->info.width));
//            gridXY.push_back(int(floor(path.at(j).index/map->info.width)));
            std::vector<double > mapXY;
            mapXY = Hastar.grid_to_coord(gridXY);
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = j;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = mapXY[0];
            marker.pose.position.y = mapXY[1];
            markerarrayx.push_back(mapXY[0]);
            markerarrayy.push_back(mapXY[1]);
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = -0.7071;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 0.7071;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
//            marker_pub.publish(marker);
            ma.markers.push_back(marker);
        }
        markerArray_pub.publish(ma);
        ma.markers.clear();
    }

}

//void publish_tf(){
//    static tf::TransformBroadcaster br;
//    tf::Transform transform;
//    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//    tf::Quaternion q;
//    q.setRPY(0, 0, 0);
//    transform.setRotation(q);
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
////    cout<<"dddffdf"<<endl;
//}
/*
void myCallback(const nav_msgs::OccupancyGrid::ConstPtr map){
    if (  std::equal(map->data.begin() + 1, map->data.end(), map->data.begin()) ){

    } else{
//        astar.my_map = *map;
//        cout<<"7346075 :"<<int(map->data.at(7346075))<<endl;
//        cout<<"1000 :"<<int(map->data[1000])<<endl;
//        cout<<"size of data :"<<map->data.size()<<endl;
//        if(map->data.at(1000) == -1 || map->data.at(1000) == 100||map->data.at(1000) == 0 ){
//            cout<<"ininii"<<endl;
//        }
//        astar.set_index_free_space();
        clock_t t1 = clock();
        astar.A_star_search();
        cout <<"used  time : "<< (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 <<" ms"<<endl;
        cout<<"size "<<astar.path.size()<<endl;
//        init_pub_markerarray(astar.path);
//        init_marker(astar.starting_point,1000);
//        init_marker(astar.goal,2000);
        pub_test.publish(astar.my_map);
//        init_pub_markerarray(astar.path);

    }


}*/
/*void myCallback_inf(const nav_msgs::OccupancyGrid::ConstPtr map) {
    if (std::equal(map->data.begin() + 1, map->data.end(), map->data.begin())) {

    } else{
//        cout<<"pppp"<<endl;

        builder.original_map = *map;
        builder.inflation_map = *map;
        builder.set_obstacle_space();
        builder.set_obstacle_edge_vec();
        builder.inflation();
        pub_inf.publish(builder.inflation_map);
        astar.my_map = builder.inflation_map;
        if(receive_goal_from_rviz==true &&receive_starting_from_rviz==true){
            //        astar.set_start_end();
//            init_marker(astar.starting_point,1000);//yellow
//            init_marker(astar.goal,100);//red
            astar.A_star_search();

//            init_pub_markerarray(astar.path);
        }



    }
}*/
void myCallback_goal_rviz(geometry_msgs::PoseStamped goal){

    if(type_from_rviz == 0){
        type_from_rviz+=1;
        receive_starting_from_rviz = true;
        double w = goal.pose.orientation.w;
        double z = goal.pose.orientation.z;
        double angle = acos(w) * 2;
        if(z < 0){
            angle  = -angle;
        }
	tmpx= goal.pose.position.x;
	tmpy = goal.pose.position.y;



    } else if (type_from_rviz == 1){
        cout<<" 111111 "<<endl;

        type_from_rviz+=1;
        receive_goal_from_rviz = true;
        vector<double> coord;
	cout<<"tmpx "<<tmpx<<endl;
	cout<<"tmpy "<<tmpx<<endl;
	Hastar.SetStartingGoalPoint(tmpx,tmpy,-90,goal.pose.position.x,goal.pose.position.y);

    } else{
        cout<<" error "<<endl;
        receive_goal_from_rviz = false;
        receive_starting_from_rviz=false;
    }

}

void myCallback_hybric(const nav_msgs::OccupancyGrid::ConstPtr map){
    if (std::equal(map->data.begin() + 1, map->data.end(), map->data.begin())) {

    } else{
        builder.newest_map = *map;
        builder.set_obstacle_space();
        builder.set_obstacle_edge_vec();
        builder.inflation();
        Hastar.my_map = builder.inflation_map;

        cout << " hhihihihihihi"<< endl;
//        Hastar.set_index_free_space();
//        Hastar.set_start_end();

        if(receive_goal_from_rviz==true &&receive_starting_from_rviz==true) {
            cout << " inininininin"<< endl;
            //rrt.build_tree();
	    Hastar.initialize();
	    Hastar.A_star_search();
            cout << " overoverover"<< endl;
            Initmarker(Hastar.starting_point,100);
            Initmarker(Hastar.goal,2000);
            pub_inf.publish(builder.inflation_map);
            cout << " size of path :" << Hastar.path.size() << endl;
            InitPubMarkerarray(Hastar.path);
     /*       for (int j = 0; j < rrtt.path.size(); ++j) {
                cout<<" vs : "<<Hastar.path[j].vs<<endl;
                cout<<" steering angle : "<<Hastar.path[j].steering_angle<<endl;
                cout<<" theta : "<<Hastar.path[j].theta<<endl;

            }*/
        }


    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_move_base");
    ros::NodeHandle n;
//    ros::Subscriber sub = n.subscribe("map", 1000, myCallback);
//    ros::Subscriber sub_inf = n.subscribe("map", 1000, myCallback_inf);
    ros::Subscriber sub_hybric = n.subscribe("map", 1000, myCallback_hybric);

    ros::Subscriber sub_goal_rviz = n.subscribe("/move_base_simple/goal", 1000, myCallback_goal_rviz);

    ros::Rate loop_rate(10);
//    Hastar = *new HybricAStar;
//    astar = *new A_star();
    Hastar = *new HybricAStar();
    builder = *new inflation_obstacle_builder();
    markerArray_pub = n.advertise<visualization_msgs::MarkerArray>("MarkerArray", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    pub_inf = n.advertise<nav_msgs::OccupancyGrid>("my_inf_map", 1000);
    pub_test = n.advertise<nav_msgs::OccupancyGrid>("my_test_map", 1000);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
//        publish_tf();

    }
    return 0;
}
