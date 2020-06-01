
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
    //front_steering params
    double x_coord_front_steering;//front_steering joint in odom
    double y_coord_front_steering;//front_steering joint in odom
    double steering_angle_base_front;//front_steering joint in base_link
    //base_link params
    double theta_base_link;//base_link in odom
}my_vehicle;

int main(int argc, char **argv){
    ros::init(argc, argv, "generate_tf");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);
    ros::Subscriber sub = n.subscribe("Pose2D", 1000, Pose2dCallback);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
//        SetFrontSteeringJointFromTF();
        PublishTF();
        std::cout<<" send "<<std::endl;

// for test ##########################
/*        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin( tf::Vector3(vehicle_base_laser_x , 0.0, vehicle_base_laser_z) );
        q.setRPY(0.0 , 0.0 , 0.0 );
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser_link"));
        std::cout<<" send "<<std::endl;*/

    }

}
void Pose2dCallback(const geometry_msgs::Pose2D pose2D){
    my_vehicle.theta_base_link = pose2D.theta;
};

void SetFrontSteeringJointFromTF(){ //TO REPLACE
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("odom", "front_steering", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("odom", "front_steering",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    my_vehicle.x_coord_front_steering = transform.getOrigin().x();
    my_vehicle.y_coord_front_steering = transform.getOrigin().y();
    // ########################
    try{
        listener.waitForTransform("base_link", "front_steering", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link", "front_steering",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);//first Roll(fixed x);second pitch(fixed y);third yaw(fixed z)
    my_vehicle.steering_angle_base_front = yaw;//right hand along axis pos direction, counterclockwise means yaw is positive

    // ########################
    try{
        listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("odom", "base_link",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    my_vehicle.theta_base_link = yaw;

    // for test #############################
/*    try{
        listener.waitForTransform("base_link", "laser_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link", "laser_link",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    std::cout<<"x_coord_base_front test: "<<transform.getOrigin().x()<<std::endl;
    std::cout<<"y_coord_base_front test: "<<transform.getOrigin().z()<<std::endl;
    std::cout<<"x test : "<<transform.getRotation().x()<<std::endl;
    std::cout<<"y test : "<<transform.getRotation().y()<<std::endl;
    std::cout<<"z test : "<<transform.getRotation().z()<<std::endl;
    std::cout<<"w test : "<<transform.getRotation().w()<<std::endl;*/


};
void PublishTF(){
    tf::TransformBroadcaster br;
    tf::Transform transform;
    //publish odom -> base_link
    double x_coord_odom_base = my_vehicle.x_coord_front_steering - cos(my_vehicle.theta_base_link) * vehicle_base_front_x ;
    double y_coord_odom_base = my_vehicle.y_coord_front_steering - sin(my_vehicle.theta_base_link) * vehicle_base_front_x ;
    transform.setOrigin( tf::Vector3(x_coord_odom_base, y_coord_odom_base, 0.0) );
    tf::Quaternion q;
    q.setRPY(0.0 , 0.0 , my_vehicle.theta_base_link );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));


    //publish base_link -> front_steering
    double x_coord_base_front = vehicle_base_front_x;
    double y_coord_base_front = 0.0;
    transform.setOrigin( tf::Vector3(x_coord_base_front,y_coord_base_front, 0.0) );
    q.setRPY(-1.57 , 0.0 , my_vehicle.steering_angle_base_front );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "front_steering"));


    //publish base_link -> laser_link
    transform.setOrigin( tf::Vector3(vehicle_base_laser_x , 0.0, vehicle_base_laser_z) );
    q.setRPY(0.0 , 0.0 , 0.0 );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser_link"));


/*    std::cout<<"x_coord_base_front: "<<transform.getOrigin().x()<<std::endl;
    std::cout<<"y_coord_base_front: "<<transform.getOrigin().z()<<std::endl;
    std::cout<<"x  : "<<transform.getRotation().x()<<std::endl;
    std::cout<<"y  : "<<transform.getRotation().y()<<std::endl;
    std::cout<<"z  : "<<transform.getRotation().z()<<std::endl;
    std::cout<<"w  : "<<transform.getRotation().w()<<std::endl;*/


};


