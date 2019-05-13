#include <freefloating_gazebo/model_control_compute.h> //This one is meant to replace the pids_body.h
//#include <freefloating_gazebo/freefloating_pids_body.h>
//#include <freefloating_gazebo/freefloating_pids_joint.h>
#include <freefloating_gazebo/thruster_allocator.h>
#include <memory>
#include <dynamic_reconfigure/server.h>
#include <freefloating_gazebo/BrovConfig.h>

using std::cout;
using std::endl;

free_floating_gazebo::BrovConfig cur_config;
bool reconfig_updated = false;

void Dyn_rqt_callback(free_floating_gazebo::BrovConfig &config, uint32_t level)
{
    cur_config = config;
    reconfig_updated = true;
}

int main(int argc, char ** argv)
{
    // init ROS node
    ros::init(argc, argv, "freefloating_model_control");
    ros::NodeHandle nh;
    ros::NodeHandle control_node(nh, "controllers");
    ros::NodeHandle priv("~");

    ffg::ThrusterAllocator allocator(nh);



    // wait for Gazebo running
    ros::service::waitForService("/gazebo/unpause_physics");
    const bool control_body = allocator.has_thrusters();

    // loop rate
    ros::Rate loop(100);
    ros::Duration dt(.01);

    ros::SubscribeOptions ops;

    // -- Init body ---------------
    // Model-Control class
    std::unique_ptr<ffg::ModelControlCompute> body_controller;
    ros::Publisher body_command_publisher;
    std::string default_mode = "velocity";

    dynamic_reconfigure::Server<free_floating_gazebo::BrovConfig> server;
    dynamic_reconfigure::Server<free_floating_gazebo::BrovConfig>::CallbackType f;
    f = boost::bind(&Dyn_rqt_callback, _1, _2);
    server.setCallback(f);

    std::stringstream ss;
    ss << "Init control for " << nh.getNamespace() << ": ";
    if(control_body)
    {
        if(priv.hasParam("body_control"))
            priv.getParam("body_control", default_mode);
        //TODO: controlled_axes relies upon finding PID parameters, so when we changed the command law, it stopped working properly
        //That's why ModelControlComputey n==0 in Init Function
        const auto controlled_axes = allocator.initControl(nh, 0.07);//Est-ce que ça change qqch dans notre loi de commande ?

        body_controller = std::unique_ptr<ffg::ModelControlCompute>(new ffg::ModelControlCompute());
        body_controller->Init(nh, dt,allocator.base_link, default_mode);

        // command
        body_command_publisher =
                nh.advertise<sensor_msgs::JointState>("thruster_command", 1);

        ss << controlled_axes.size() << " controlled axes (" << default_mode << " control)";
    }

    while(ros::ok())
    {

        // update body and publish
        if(control_body && body_controller->Update(cur_config, reconfig_updated))
        {
            body_command_publisher.publish(allocator.wrench2Thrusters(body_controller->WrenchCommand()));
        }

        ros::spinOnce();
        loop.sleep();
    }
}
