
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#define vehicle_base_front_x 0.75*3/4
#define vehicle_base_laser_x 0.75-0.04479+0.01679
#define vehicle_base_laser_z 0.06+0.6
void Pose2dCallback(const geometry_msgs::Pose2D pose2D);
void SetFrontSteeringJointFromTF();
void PublishTF();

struct vehicle{
    //front_steering param
    double x_coord_front_steering;//front_steering joint in odom
    double y_coord_front_steering;//front_steering joint in odom
    double steering_angle_base_front;//front_steering joint in base_link
    //base_link params
    double theta_base_link;//base_link in odom
}my_vehicle;

int main(int argc, char **argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        tf::TransformListener listener;
        tf::StampedTransform transform1;
        try{
            listener.waitForTransform("/base_link", "/front_steering", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/front_steering",
                                     ros::Time(0), transform1);
            std::cout<<" get "<<transform1.getOrigin().x()<<std::endl;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        std::cout<<" listen "<<std::endl;

    }

}





