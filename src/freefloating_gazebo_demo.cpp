#include <freefloating_gazebo/BodySetpoint.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

using std::cout;
using std::endl;

int main(int argc, char ** argv)
{
    // this node
    ros::init(argc, argv, "auv_robot_demo_control");
    ros::start();
    ros::NodeHandle ros_node;
    const double rate = 30;
    ros::Rate loop(rate);

    freefloating_gazebo::BodySetpoint body_setpoint_;
    body_setpoint_.pose.position.x = 5.3;
    body_setpoint_.pose.position.y = 6.7;
    body_setpoint_.pose.position.z = -11.3;
    body_setpoint_.pose.orientation.x = 1;
    body_setpoint_.pose.orientation.y = 1;
    body_setpoint_.pose.orientation.z = 0;
    body_setpoint_.pose.orientation.w = 0;

    ros::Publisher current_publisher = ros_node.advertise<freefloating_gazebo::BodySetpoint>("/body_setpoint", 1);  
    while (ros::ok())
    {

    ROS_INFO("Publishing body_setpoint_");
    current_publisher.publish(body_setpoint_);

    }
}
