//
// Created by parallels on 5/2/20.
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
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include <tf/transform_listener.h>
#include "../include/my_move_base/pid.h"
#include <time.h>
#include <Eigen/Dense>
#define pi 3.1415926
#define carlength 0.75
using namespace std;
void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr map);//subscribe topic map
void FrontierCallback(const geometry_msgs::Point frontier );//subscribe topic frontier
void PublishCmd();//publish /cmd_vel to follow a trajectory
vector<std::vector<double>> GetPosOri(string focuslink);
std::vector<int>  coord_to_grid(std::vector<double> coord, nav_msgs::OccupancyGrid* map);
vector<std::vector<double>> GetHeadPosOri(vector<std::vector<double>> currentposori);
double GetThetaFromquaternion(vector<double> quaternion);//get the angle between two x axis
void FillMap(nav_msgs::OccupancyGrid* map);
void InitStartingGoalPoint();
void Rotate(double durationref);
void PublishNoPathFound();
bool PointInObstacle(double coordx,double coordy);
void Initmarker(A_star::cell cell ,int id);//for test
void InitPubMarkerarray(vector<A_star::cell> path);//for test

A_star astar;
inflation_obstacle_builder builder;
ros::Publisher pubcmdvel ;
ros::Publisher pubfinished ;// published when the rotor reached the current goal pos and finish a rotation
ros::Publisher pubfilledmap;
ros::Publisher closelistmap;
ros::Publisher marker_array_pub;//publish waypoints
ros::Publisher marker_pub;//publish starting and goal point
ros::Publisher pub_no_path_found;
ros::Publisher pubinflationmap;
Pid p;
//mutex mtx;
bool newfrontierreceived = false;//new frontier comes
bool firstmapstored = false;//at least one map is stored
bool sent = false;//comment below
bool reachgoal = true;// current trajectory finishes following when true
double goalxcoord = 0;
double goalycoord = 0;
std::vector<double> markerarrayx;// for display
std::vector<double> markerarrayy;//for display
double StartingPosforFillingx = 0;//record the coord of the starting pos of the whole program for filling the map
double StartingPosforFillingy = 0;
bool first_time_running = true;
//clock_t clock_following_one_point ;//if take too long to follow a point in the path, skip the point
//double duration_following_one_point =0;
int main(int argc, char **argv){
    ros::init(argc, argv, "move_base_astar");
    ros::NodeHandle n;
    ros::Subscriber mapsubscriber = n.subscribe("map", 1000, MapCallback);
    ros::Subscriber frontiersubscriber = n.subscribe("frontier", 1000, FrontierCallback);
    pubcmdvel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);// publish the /cmd_vel to control a car
    pubfinished = n.advertise<std_msgs::Bool>("/finished", 1000);// publish the /finished to inform exploration package
    pub_no_path_found = n.advertise<std_msgs::Bool>("/no_path_found", 1000);//in exploration package when no path found
    pubfilledmap = n.advertise<nav_msgs::OccupancyGrid>("filled_map", 1000);//
    closelistmap = n.advertise<nav_msgs::OccupancyGrid>("close_list_map", 1000);//
    pubinflationmap =  n.advertise<nav_msgs::OccupancyGrid>("inflation_map", 1000);//
    marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("MarkerArray", 10);//for test
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);//for test

    ros::Rate loop_rate(50);
    astar = *new A_star;
    builder = *new inflation_obstacle_builder();
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();

        if(newfrontierreceived == true){// search only when new frontier comes

            if(firstmapstored == false ){ continue;}//search only when map exists
            cout<<"########## new round start ############"<<endl;
            builder.set_obstacle_space();
            builder.set_obstacle_edge_vec();
            builder.inflation();
            astar.my_map = builder.inflation_map;
            astar.close_list_map_ = astar.my_map;
            pubinflationmap.publish(builder.inflation_map);
            InitStartingGoalPoint();
            Initmarker(astar.starting_point,10001);//
            Initmarker(astar.goal,10002);
            astar.A_star_search();
            closelistmap.publish(astar.close_list_map_);
            if(astar.search_success_ == false){
                PublishNoPathFound();
//                sent = false;// in order to publish finished
            }else{
                for(int i = 0 ; i < astar.path.size() ; i++){
                    int gridXY[2]= {astar.path[i].x,astar.path[i].y};
                    vector<double> coordXY = astar.grid_to_coord(gridXY);
                    cout<<" path "<<i<<" : "<<coordXY[0]<<","<<coordXY[1]<<endl;
                }
            }
            InitPubMarkerarray(astar.path);
            newfrontierreceived = false;
            reachgoal = false;
        }
        PublishCmd();

    }
    return 0;
}
void FillMap(nav_msgs::OccupancyGrid* map){
    if(first_time_running == true){// only fill the starting position , which is different from the frontier package
        vector<std::vector<double>> currentposori = GetPosOri("base_link");
        StartingPosforFillingx = currentposori[0][0];
        StartingPosforFillingy = currentposori[0][1];
        first_time_running = false;

    }
    vector<double> coordXY;
    coordXY.push_back(StartingPosforFillingx);
    coordXY.push_back(StartingPosforFillingy);
    vector<int> mapXY = coord_to_grid(coordXY,map);

    for(int i = -4; i <= 4 ; i++){
        for(int j = -4 ; j <= 4; j++){
            map->data[(mapXY[0]+i) + (mapXY[1]+j)*map->info.width] = 0;
        }
    }
    return;

}
void PublishNoPathFound(){
    std_msgs::Bool msg;
    msg.data = true;
    pub_no_path_found.publish(msg);
}
void InitStartingGoalPoint(){
    vector<std::vector<double>> currentposori = GetPosOri("base_link");
    vector<std::vector<double>> headposori = GetHeadPosOri(currentposori);

    double coordx = headposori[0][0];
    double coordy = headposori[0][1];

    int rotatecount = 0;
    while(PointInObstacle(coordx,coordy)){// prevent from setting the starting point in the obstacle
        cout<<"starting point in the obstacle; adjusting ... "<<endl;
        Rotate(0.174);//10 degree

        vector<std::vector<double>> currentposori = GetPosOri("base_link");
        vector<std::vector<double>> headposori = GetHeadPosOri(currentposori);

        coordx = headposori[0][0];
        coordy = headposori[0][1];
        rotatecount++;
        if(rotatecount > 50){
            cout<<"all starting points are in the obstacle "<<endl;
            exit(0);
        }
    }
    geometry_msgs::Twist msgstop;
    msgstop.linear.x = 0;
    msgstop.linear.y = 0;
    msgstop.linear.z = 0;
    msgstop.angular.x = 0;
    msgstop.angular.y = 0;
    msgstop.angular.z = 0;
    pubcmdvel.publish(msgstop);

    astar.SetStartingGoalPoint(coordx,coordy ,goalxcoord,goalycoord);
    cout<<" set starting point : "<<coordx<<" , "<<coordy<<endl;
    cout<<" set ending point : "<<goalxcoord<<" , "<<goalycoord<<endl;

}
bool PointInObstacle(double coordx,double coordy){// true means point in obstacle

    vector<double> coordxy;
    coordxy.push_back(coordx);
    coordxy.push_back(coordy);
    vector<int> gridxy =astar.coord_to_grid(coordxy);
    return builder.inflation_map.data[gridxy[0]+gridxy[1]*builder.inflation_map.info.width] == 100 ||
           builder.inflation_map.data[gridxy[0]+gridxy[1]*builder.inflation_map.info.width] == 50;
}
//because the area under the car is undetectable, so the starting point is not
//selected to be the footprint/baseline, but the point at the head of the car
vector<std::vector<double>> GetHeadPosOri(vector<std::vector<double>> currentposori){
    double theta_s ;
    theta_s = GetThetaFromquaternion(currentposori[1]);
    double deltax = cos(theta_s) * carlength;//0.999
    double deltay = sin(theta_s) * carlength;//-0.02
    cout<<"deltax : "<<deltax<<endl;
    cout<<"deltay : "<<deltay<<endl;
    vector<std::vector<double>> res;
    vector<double> respos;
    respos.push_back(currentposori[0][0] + deltax);//5.17 + 0.999
    respos.push_back(currentposori[0][1] + deltay);//-2.62 - 0.02
    res.push_back(respos);
    res.push_back(currentposori[1]);
    return res;
}

double GetThetaFromquaternion(vector<double> quaternion){
    double theta ;
    if (quaternion[2] > 0){ // [x y z w ] -->  z > 0
        theta = acos(quaternion.back()) * 2; //rad -->w
    } else{
        theta = - acos(quaternion.back()) * 2; //rad
    }
    cout<<"theta : "<<theta<<endl;
    return theta;
}


void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr map){
    if (std::equal(map->data.begin() + 1, map->data.end(), map->data.begin())) {//avoid no empty map
        return;
    }
    builder.newest_map = *map;
    FillMap(&builder.newest_map);
    pubfilledmap.publish(builder.newest_map);
    firstmapstored = true;

    return;

}
void FrontierCallback(const geometry_msgs::Point frontier){

    goalxcoord = frontier.x;
    goalycoord = frontier.y;
    newfrontierreceived = true;
}

void Rotate(double rotateangle ){

    double linearvel = 0.2;
    double durationref = abs(rotateangle) * (0.75 * 3 / 4) /linearvel;
    double rotatevel = rotateangle > 0 ? -1.57 : 1.57;
    clock_t ts = clock();
    double duration = (clock() - ts) * 1.0 / CLOCKS_PER_SEC ;
    while(duration <= durationref){
        geometry_msgs::Twist msg;
        msg.linear.x = linearvel;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = rotatevel;
        pubcmdvel.publish(msg);
        duration = (clock() - ts) * 1.0 / CLOCKS_PER_SEC ;
    }

}

void PublishCmd(){

    if(reachgoal == true ){//run when it reaches the end of path
        if(sent == true){ return;}// just publish once when path is empty
        cout<<"start scanning ..."<<endl;
        Rotate(0.785);//countwise 45 degree
        Rotate(-1.57);//entercountwise 90 degree
        Rotate(0.785);//countwise 45 degree
        geometry_msgs::Twist msgstop;
        msgstop.linear.x = 0;
        msgstop.linear.y = 0;
        msgstop.linear.z = 0;
        msgstop.angular.x = 0;
        msgstop.angular.y = 0;
        msgstop.angular.z = 0;
        pubcmdvel.publish(msgstop);
        cout<<"end scanning "<<endl;

        std_msgs::Bool msg;
        msg.data = true;
        pubfinished.publish(msg);// no matter the path can be found or not, finished topic should be sent
        cout<<"******* reached next frontier ******"<<endl;
        sent = true;
        return;
    }
    if(astar.search_success_ == false){ return;}
    sent = false;// once start following, reset
    A_star::cell& goal_cell = astar.path.back();//first element of path is the final goal
    vector<double> goalpos;
    int gridXY[2]= {goal_cell.x,goal_cell.y};
    vector<double> coordXY = astar.grid_to_coord(gridXY);
    goalpos.push_back(coordXY[0]);
    goalpos.push_back(coordXY[1]);
    vector<vector<double>> frontsteeringposori = GetPosOri("front_steering");
    vector<vector<double>> baselinkposori = GetPosOri("base_link");// used for skip the waypoints under and right behind the car
    vector<double> res = p.SetVel(baselinkposori,frontsteeringposori,goalpos);

    if (res[2] == 1.0){// if car reached the current goal
        astar.path.pop_back();
        if(astar.path.size() == 0){
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
            pubcmdvel.publish(msg);
            reachgoal = true;
        }
        cout<<" *****reach point***** :"<<coordXY[0]<<", "<<coordXY[1]<<endl;
    }else{
        if(res[0] == 0){cout<<"warning!!"<<endl;}//for test
        geometry_msgs::Twist msg;
        msg.linear.x = res[0];
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = res[1];
        pubcmdvel.publish(msg);
    };
    return;
}
vector<std::vector<double>> GetPosOri(string focuslink){
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
void Initmarker(A_star::cell cell ,int id){
    int gridXY[2] = {cell.x , cell.y};
    std::vector<double > mapXY;
    mapXY = astar.grid_to_coord(gridXY);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
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
}

void InitPubMarkerarray(vector<A_star::cell> path){
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
    marker_array_pub.publish(ma);
    ma.markers.clear();
    markerarrayx.clear();
    markerarrayy.clear();

    if(path.size() == 0){
        std::cout<<"no path"<<std::endl;
    }else{
        visualization_msgs::MarkerArray ma;
        for (int j = 0; j < path.size(); ++j) {
            int  gridXY[2] = {int((path.at(j).index + astar.my_map.info.width)%astar.my_map.info.width),
                              int(floor(path.at(j).index/astar.my_map.info.width))};
            std::vector<double > mapXY;
            mapXY = astar.grid_to_coord(gridXY);
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
            ma.markers.push_back(marker);
        }
        marker_array_pub.publish(ma);
        ma.markers.clear();
    }

}
std::vector<int>  coord_to_grid(std::vector<double> coord, nav_msgs::OccupancyGrid* map){
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.y;
    double origin_z = map->info.origin.position.z;
    double q1 = map->info.origin.orientation.x;
    double q2 = map->info.origin.orientation.y;
    double q3 = map->info.origin.orientation.z;
    double q0 = map->info.origin.orientation.w;

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
    grid_xy = H.inverse() * map_xy / map->info.resolution;

    vector<int > gridXY;
    gridXY.push_back(int(grid_xy(0,0)));
    gridXY.push_back(int(grid_xy(1,0)));

    return gridXY;
}