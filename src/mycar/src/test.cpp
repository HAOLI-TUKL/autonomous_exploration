#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <sstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped pose;

        chatter_pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
