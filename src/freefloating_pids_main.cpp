#include <freefloating_gazebo/freefloating_pids_body.h>
#include <freefloating_gazebo/freefloating_pids_joint.h>
#include <freefloating_gazebo/thruster_allocator.h>
#include <memory>

using std::cout;
using std::endl;

int main(int argc, char ** argv)
{
  // init ROS node
  ros::init(argc, argv, "freefloating_pid_control");
  ros::NodeHandle nh;
  ros::NodeHandle control_node(nh, "controllers");
  ros::NodeHandle priv("~");

  ffg::ThrusterAllocator allocator(nh);

  // wait for Gazebo running
  ros::service::waitForService("/gazebo/unpause_physics");
  const bool control_body = allocator.has_thrusters();
  const bool control_joints = FreeFloatingJointPids::writeJointLimits(nh);

  // loop rate
  ros::Rate loop(100);
  ros::Duration dt(.01);

  ros::SubscribeOptions ops;

  // -- Init body ------------------
  // PID's class
  std::unique_ptr<FreeFloatingBodyPids> body_pid;
  ros::Publisher body_command_publisher;
  std::string default_mode = "position";

  std::stringstream ss;
  ss << "Init PID control for " << nh.getNamespace() << ": ";
  if(control_body)
  {
    if(priv.hasParam("body_control"))
      priv.getParam("body_control", default_mode);

    const auto controlled_axes = allocator.initControl(nh, 0.07);
    body_pid = std::unique_ptr<FreeFloatingBodyPids>(new FreeFloatingBodyPids);
    body_pid->Init(nh, dt, controlled_axes, default_mode);

    // command
    body_command_publisher =
        nh.advertise<sensor_msgs::JointState>("thruster_command", 1);

    ss << controlled_axes.size() << " controlled axes (" << default_mode << " control)";
  }

  // -- Init joints ------------------
  std::unique_ptr<FreeFloatingJointPids> joint_pid;
  if(control_joints)
  {
    default_mode = "position";
    if(priv.hasParam("joint_control"))
      priv.getParam("joint_control", default_mode);

    joint_pid = std::unique_ptr<FreeFloatingJointPids>(new FreeFloatingJointPids);
    joint_pid->Init(nh, dt, default_mode);
  }

  std::vector<std::string> joint_names;
  if(control_joints)
  {
    control_node.getParam("config/joints/name", joint_names);
    ss << ", " << joint_names.size() << " joints (" << default_mode << " control)";
  }

  ROS_INFO("%s", ss.str().c_str());

  while(ros::ok())
  {
    // update body and publish
    if(control_body && body_pid->UpdatePID())
        body_command_publisher.publish(allocator.wrench2Thrusters(body_pid->WrenchCommand()));

    // update joints and publish
    if(control_joints && joint_pid->UpdatePID())
      joint_pid->publish();

    ros::spinOnce();
    loop.sleep();
  }
}
