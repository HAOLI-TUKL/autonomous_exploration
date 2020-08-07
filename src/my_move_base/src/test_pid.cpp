//
// Created by parallels on 4/18/20.
//
#include "ros/ros.h"
#include "../include/my_move_base/pid.h"
#include "../include/my_move_base/HybricAStar.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define carlength 0.75 * 3.0 /4.0
using namespace std;
void publish_cmd();
void set_path();
vector<std::vector<double>> GetPosOri(string focuslink);
double GetThetaFromquaternion(vector<double> quaternion);
vector<std::vector<double>> GetHeadPosOri(vector<std::vector<double>> currentposori);
void InitPubMarkerarray(vector<HybricAStar::cell> path);
void Initmarker(HybricAStar::cell cell ,int id);
Pid p;
vector<HybricAStar::cell> path;
ros::Publisher pubcmdvel ;
ros::Publisher marker_array_pub;
HybricAStar Hastar;
bool sent = false;



int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pid");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);// the faster ,the better
    pubcmdvel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("MarkerArray", 1000);//for test
//sleep(5);
    int go;
    cout<<"go ? ";
    cin>>go;
    Hastar = *new HybricAStar;
    set_path();

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if (sent == false){
            InitPubMarkerarray(path);
        }
        publish_cmd();

    }
    return 0;
}
void publish_cmd(){
    if(path.size() == 0){ return;}
    HybricAStar::cell& goal_cell = path.back();//first element of path is the final goal
    vector<double> goalpos;
    goalpos.push_back(goal_cell.x_coord);
    goalpos.push_back(goal_cell.y_coord);
    //cout<<" going to point : "<<goal_cell.x_coord<<", "<<goal_cell.y_coord<<endl;
//    vector<vector<double>> baselinkpoint = GetPosOri("base_link");
//    vector<std::vector<double>> headposori = GetHeadPosOri(baselinkpoint);
    vector<vector<double>> frontsteeringposori = GetPosOri("front_steering");
    vector<vector<double>> baselinkposori = GetPosOri("base_link");
    vector<double> res = p.SetVel(baselinkposori,frontsteeringposori,goalpos);//TODO
/**
    geometry_msgs::Twist msg;
    msg.linear.x = res[0];
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = res[1];
    pubcmdvel.publish(msg);
    if (res[2] == 1.0){
        path.pop_back();
        cout<<" reach point :"<<goal_cell.x_coord<<", "<<goal_cell.y_coord<<endl;
    };// if car reached the current goal
     
**/


    if (res[2] == 1.0){// if car reached the current goal
        path.pop_back();
        if(path.size() == 0){
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
            pubcmdvel.publish(msg);
          //  reachgoal = true;
            cout<<"****** reach goal !!!!!*******"<<endl;
        }
        cout<<" reach point :"<<goal_cell.x_coord<<", "<<goal_cell.y_coord<<endl;
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

//because the area under the car is undetectable, so the starting point is not
//selected to be the footprint/baseline, but the point at the head of the car
vector<std::vector<double>> GetHeadPosOri(vector<std::vector<double>> currentposori){
    double theta_s ;
    theta_s = GetThetaFromquaternion(currentposori[1]);
    double deltax = cos(theta_s) * carlength;
    double deltay = sin(theta_s) * carlength;
    vector<std::vector<double>> res;
    vector<double> respos;
    respos.push_back(currentposori[0][0] + deltax);
    respos.push_back(currentposori[0][1] + deltay);
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
    return theta;
}

void set_path(){// manually generate a trajectory for test
  HybricAStar::cell goal3 = *new HybricAStar::cell();
     /*   goal3.x_coord = 3.5;
        goal3.y_coord = 1.5;
        path.push_back(goal3);

        HybricAStar::cell goal2 = *new HybricAStar::cell();
        goal2.x_coord = 2.6;
        goal2.y_coord = 1;
        path.push_back(goal2);

	        HybricAStar::cell goal1 = *new HybricAStar::cell();
        goal1.x_coord = 2;
        goal1.y_coord = 0.5;
	path.push_back(goal1);*/
        HybricAStar::cell goal = *new HybricAStar::cell();
        goal.x_coord = -2;
        goal.y_coord = 0;
        path.push_back(goal);
  /**
	HybricAStar::cell goal1 = *new HybricAStar::cell();
        goal1.x_coord = -2.5;
        goal1.y_coord = 0;
        path.push_back(goal1);
	HybricAStar::cell goal2 = *new HybricAStar::cell();
        goal2.x_coord = -2.5;
        goal2.y_coord = 1;
        path.push_back(goal2);
	        HybricAStar::cell goal3 = *new HybricAStar::cell();
        goal3.x_coord = -2.5;
        goal3.y_coord = -1;
        path.push_back(goal3);
**/
 /*   for(int i = 8; i >= 0 ;i--){
        HybricAStar::cell goal = *new HybricAStar::cell();
        if((i % 2) == 0){
            goal.x_coord = 1;
        } else{
            goal.x_coord = -1;

        }
        goal.y_coord = i + 1 ;
        path.push_back(goal);

    }*/

//    cout<<"path size :"<<path.size()<<endl;
/**	
    for(double  i = 6.28; i >= 0 ;){
        A_star::cell goal = *new A_star::cell();
        goal.x_coord = sin(i) * 2;
        goal.y_coord = i  ;
        path.push_back(goal);
        i-=0.2;

    }
**/

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

void InitPubMarkerarray(vector<HybricAStar::cell> path) {

    if (path.size() == 0) {
    } else {
//        cout << "1" << endl;
        visualization_msgs::MarkerArray ma;
        for (int j = 0; j < path.size(); ++j) {

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = j;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
//            cout<<"x "<<path[j].x_coord<<endl;
//            cout<<"y "<<path[j].y_coord<<endl;

            marker.pose.position.x = path[j].x_coord;
            marker.pose.position.y = path[j].y_coord;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = -0.7071;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 0.7071;
            marker.scale.x = 0.07;
            marker.scale.y = 0.07;
            marker.scale.z = 0.07;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            ma.markers.push_back(marker);

        }
        marker_array_pub.publish(ma);
//        cout << "2" << endl;
        ma.markers.clear();

    }
}
