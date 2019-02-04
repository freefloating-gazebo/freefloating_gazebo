#include <freefloating_gazebo/model_control_compute.h>  //This one is meant to replace the pids_body.h
#include <freefloating_gazebo/freefloating_pids_body.h>
#include <freefloating_gazebo/freefloating_pids_joint.h>
#include <freefloating_gazebo/thruster_allocator.h>
#include <memory>

using std::cout;
using std::endl;

int main(int argc, char ** argv)
{
    // init ROS node
    ros::init(argc, argv, "freefloating_model_control");
    ros::NodeHandle nh;
    ros::NodeHandle control_node(nh, "controllers");//Was used whaen we needed to Init Joints
    ros::NodeHandle priv("~");

    ffg::ThrusterAllocator allocator(nh);

    // wait for Gazebo running
    ros::service::waitForService("/gazebo/unpause_physics");
    const bool control_body = allocator.has_thrusters();

    // loop rate
    ros::Rate loop(100);
    ros::Duration dt(.01);

    ros::SubscribeOptions ops;

    // -- Init body --------------------------
    // Model-Control class
    std::unique_ptr<ffg::ModelControlCompute> body_controller;
    ros::Publisher body_command_publisher;
    std::string default_mode = "position";

    std::stringstream ss;
    ss << "Init control for " << nh.getNamespace() << ": ";
    if(control_body)
    {
        if(priv.hasParam("body_control"))
            priv.getParam("body_control", default_mode);

        const auto controlled_axes = allocator.initControl(nh, 0.07);

        body_controller = std::unique_ptr<ffg::ModelControlCompute>(new ffg::ModelControlCompute());
        //std::make_unique<ffg::ModelControlCompute>();
        //body_controller->Init(nh, dt, controlled_axes, default_mode); It was needed before, now we do it when we assign the pointer
        //body_pid->Init(nh, dt, controlled_axes, default_mode);

        // command
        body_command_publisher =
                nh.advertise<sensor_msgs::JointState>("thruster_command", 1);

        ss << controlled_axes.size() << " controlled axes (" << default_mode << " control)";
    }


    ROS_INFO("%s", ss.str().c_str());

    while(ros::ok())
    {
        // update body and publish
        if(control_body && body_controller->Update())
            body_command_publisher.publish(allocator.wrench2Thrusters(body_controller->WrenchCommand()));//Write fonction for Control Compute


        // // update joints and publish
        //if(control_joints && joint_pid->UpdatePID())
        //joint_pid->publish();

        ros::spinOnce();
        loop.sleep();
    }
}
