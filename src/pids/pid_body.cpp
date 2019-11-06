#include <freefloating_gazebo/freefloating_pids_body.h>
#include <freefloating_gazebo/thruster_allocator.h>

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

  if(!allocator.has_thrusters())
    return 0;

  // wait for Gazebo running
  ros::service::waitForService("/gazebo/unpause_physics");

  // loop rate
  ros::Rate loop(100);
  ros::Duration dt(.01);

  ros::SubscribeOptions ops;

  // -- Init body ------------------
  // PID's class
  FreeFloatingBodyPids body_pid;
  ros::Publisher body_command_publisher;
  std::string default_mode = "position";


  std::stringstream ss;
  ss << "Init PID control for " << nh.getNamespace() << ": ";
  if(priv.hasParam("body_control"))
    priv.getParam("body_control", default_mode);

  const auto controlled_axes = allocator.initControl(nh, 0.07);
  body_pid.Init(nh, dt, controlled_axes, default_mode);

  // command
  body_command_publisher =
      nh.advertise<sensor_msgs::JointState>("thruster_command", 1);

  ss << controlled_axes.size() << " controlled axes (" << default_mode << " control)";

  ROS_INFO("%s", ss.str().c_str());

  while(ros::ok())
  {
    // update body and publish
    if(body_pid.UpdatePID())
      body_command_publisher.publish(allocator.wrench2Thrusters(body_pid.WrenchCommand()));

    ros::spinOnce();
    loop.sleep();
  }
}
