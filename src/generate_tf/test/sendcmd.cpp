
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
ros::Publisher pubcmdvel ;

int main(int argc, char **argv){
    ros::init(argc, argv, "sendcmd");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    pubcmdvel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);// publish the /cmd_vel to
    float vel = 0;
    float angle = 0;
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        std::cout<<"input target vel : ";
        std::cin>> vel;
        std::cout<<"input target angle (rad) : ";
        if(vel == -999){ break;}
        std::cin>> angle;
        geometry_msgs::Twist t;
        t.linear.x = vel;
        t.angular.z = angle;
        pubcmdvel.publish(t);
        std::cout<<"__________ send __________ "<<std::endl;

    }

}





