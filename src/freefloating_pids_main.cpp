
#include <freefloating_gazebo/freefloating_pids_body.h>
#include <freefloating_gazebo/freefloating_pids_joint.h>

using std::cout;
using std::endl;

int main(int argc, char ** argv)
{
    // wait for params
    sleep(5);

    // init ROS node
    ros::init(argc, argv, "freefloating_pid_control");
    ros::NodeHandle rosnode;
    ros::NodeHandle pid_node(rosnode, "controllers");

    // -- Parse body data if needed ---------

    // body setpoint and state topics
    std::string body_setpoint_topic, body_state_topic = "body_state";
    std::vector<std::string> controlled_axes;
    bool control_body = false;
    ROS_INFO("Init PID control for %s", rosnode.getNamespace().c_str());
    if(pid_node.hasParam("config/body"))
    {
        control_body = true;
        pid_node.param("config/body/setpoint", body_setpoint_topic, std::string("body_setpoint"));
        pid_node.param("config/body/state", body_state_topic, std::string("body_state"));
        // controlled body axes
        pid_node.getParam("config/body/axes", controlled_axes);
    }

    // -- Parse joint data if needed ---------
    std::string joint_setpoint_topic, joint_state_topic;
    bool control_joints = false;
    if(pid_node.hasParam("config/joints/name"))
    {
        control_joints = true;
        // joint setpoint and state topics
        pid_node.param("config/joints/setpoint", joint_setpoint_topic, std::string("joint_setpoint"));
        pid_node.param("config/joints/state", joint_state_topic, std::string("joint_states"));
    }
    // -- end parsing parameter server

    // loop rate
    ros::Rate loop(100);
    ros::Duration dt(.01);

    ros::SubscribeOptions ops;

    // -- Init body ------------------
    // PID's class
    FreeFloatingBodyPids body_pid;
    ros::Subscriber body_setpoint_subscriber, body_state_subscriber;
    ros::Publisher body_command_publisher;
    if(control_body)
    {
        body_pid.Init(pid_node, dt, controlled_axes);

        // setpoint
        body_setpoint_subscriber =
                rosnode.subscribe(body_setpoint_topic, 1, &FreeFloatingBodyPids::SetpointCallBack, &body_pid);
        // measure
        body_state_subscriber =
                rosnode.subscribe(body_state_topic, 1, &FreeFloatingBodyPids::MeasureCallBack, &body_pid);
        // command
        body_command_publisher =
                rosnode.advertise<geometry_msgs::Wrench>("body_command", 1);
    }

    // -- Init joints ------------------
    FreeFloatingJointPids joint_pid;
    // declare subscriber / publisher
    ros::Subscriber joint_setpoint_subscriber, joint_state_subscriber;
    ros::Publisher joint_command_publisher;

    if(control_joints)
    {
        // pid
        joint_pid.Init(pid_node, dt);

        // setpoint
        joint_setpoint_subscriber = rosnode.subscribe(joint_setpoint_topic, 1, &FreeFloatingJointPids::SetpointCallBack, &joint_pid);

        // measure
        joint_state_subscriber = rosnode.subscribe(joint_state_topic, 1, &FreeFloatingJointPids::MeasureCallBack, &joint_pid);

        // command
        joint_command_publisher = rosnode.advertise<sensor_msgs::JointState>("joint_command", 1);
    }


    while(ros::ok())
    {
        // update body and publish
        if(control_body)
            if(body_pid.UpdatePID())
                body_command_publisher.publish(body_pid.WrenchCommand());

        // update joints and publish
        if(control_joints)
            if(joint_pid.UpdatePID())
                joint_command_publisher.publish(joint_pid.EffortCommand());

        ros::spinOnce();
        loop.sleep();
    }
}
