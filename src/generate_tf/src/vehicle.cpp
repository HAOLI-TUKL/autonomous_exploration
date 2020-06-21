#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"


ros::Publisher pubcmdvel ;
void encoderCallback(const geometry_msgs::Pose2D pose2D);
struct vehicle{
    //front_steering params
    double x_coord_front_steering;//front_steering joint in odom
    double y_coord_front_steering;//front_steering joint in odom
}my_vehicle;

int main(int argc, char **argv){
    ros::init(argc, argv, "vehicle");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    ros::Subscriber sub = n.subscribe("Pose2D", 1000, encoderCallback);
    pubcmdvel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);// publish the /cmd_vel to
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

}
void encoderCallback(const geometry_msgs::Pose2D pose2D){
    //0  = k(-75) + b; 90 = k(8000) + b; 90 = 8075k;k = 0.01115;b = 0.83; y = 0.01115x + 0.83
    double x_delta = pose2D.x * cos(pose2D.y);
    double y_delta = pose2D.x * sin(pose2D.y);
    my_vehicle.x_coord_front_steering += x_delta;
    my_vehicle.y_coord_front_steering -= y_delta;
    std::cout<<"x_coord_front_steering : "<<my_vehicle.x_coord_front_steering<<std::endl;
    std::cout<<"y_coord_front_steering : "<<my_vehicle.y_coord_front_steering<<std::endl;

}