#include <freefloating_gazebo/freefloating_pids_joint.h>

using std::cout;
using std::endl;

int main(int argc, char ** argv)
{
  // init ROS node
  ros::init(argc, argv, "pid_joints");
  ros::NodeHandle nh;
  ros::NodeHandle control_node(nh, "controllers");
  ros::NodeHandle priv("~");

  // wait for Gazebo running
  //ros::service::waitForService("/gazebo/unpause_physics");

  if(!FreeFloatingJointPids::writeJointLimits(nh))
    return 0;

  // loop rate
  ros::Rate loop(100);
  ros::Duration dt(.01);

  ros::SubscribeOptions ops;

  // -- Init joints ------------------

  std::string default_mode = "position";
  if(priv.hasParam("joint_control"))
    priv.getParam("joint_control", default_mode);

  FreeFloatingJointPids joint_pid;
  joint_pid.Init(nh, dt, default_mode);

  std::vector<std::string> joint_names;
  control_node.getParam("config/joints/name", joint_names);
  std::stringstream ss;
  ss << ", " << joint_names.size() << " joints (" << default_mode << " control)";

  ROS_INFO("%s", ss.str().c_str());

  while(ros::ok())
  {
    // update joints and publish
    if(joint_pid.UpdatePID())
      joint_pid.publish();

    ros::spinOnce();
    loop.sleep();
  }
}
