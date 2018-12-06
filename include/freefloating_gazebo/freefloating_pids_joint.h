#ifndef FREEFLOATINGJOINTPID_H
#define FREEFLOATINGJOINTPID_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>
#include <freefloating_gazebo/freefloating_pids.h>

class FreeFloatingJointPids : public FreeFloatingPids
{

public:
  void Init(ros::NodeHandle &nh,
            const ros::Duration &_dt,
            std::string default_mode = "position");

  // parse received body setpoint
  void SetpointCallBack(const sensor_msgs::JointStateConstPtr& _msg);
  // parse received body measure
  void MeasureCallBack(const sensor_msgs::JointStateConstPtr& _msg);

  // update PID's
  bool UpdatePID();

  void publish()
  {
    command_publisher.publish(joint_command);
  }

  // parse URDF and write joint limits as PID clamps
  static bool writeJointLimits(ros::NodeHandle & nh);

private:

  ros::Subscriber setpoint_subscriber, state_subscriber;
  ros::Publisher command_publisher;

  std::vector<double> joint_lower, joint_upper, joint_max_velocity;
  std::vector<double> position_error, velocity_error,
  position_filtered_measure, velocity_filtered_measure,
  position_measure, velocity_measure ;
  sensor_msgs::JointState joint_setpoint, joint_command;
  bool vmax_is_set;
  double alpha;
};

#endif // FREEFLOATINGJOINTPID_H
