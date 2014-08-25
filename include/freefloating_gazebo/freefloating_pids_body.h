#ifndef FREEFLOATINGBODYPID_H
#define FREEFLOATINGBODYPID_H

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <freefloating_gazebo/BodySetpoint.h>
#include <control_toolbox/pid.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <freefloating_gazebo/freefloating_pids.h>

class FreeFloatingBodyPids : public FreeFloatingPids
{

public:
    void Init(const ros::NodeHandle &_node, ros::Duration&_dt, const std::vector<std::string>&_controlled_axes);

    // parse received body setpoint
    void SetpointCallBack(const freefloating_gazebo::BodySetpointConstPtr & _msg);
    // parse received body measure
    void MeasureCallBack(const nav_msgs::OdometryConstPtr& _msg);

    // update PID's
    bool UpdatePID();

    // get wrench command
    inline geometry_msgs::Wrench WrenchCommand() {return wrench_command_;}

private:

    // errors are stored in Vector3
    Eigen::Vector3d pose_lin_error_, pose_ang_error_, velocity_lin_error_, velocity_ang_error_;
    // velocities also
    Eigen::Vector3d velocity_lin_setpoint_, velocity_ang_setpoint_, velocity_lin_measure_, velocity_ang_measure_;
    // poses are stored in Vector3 and Quaternion
    Eigen::Vector3d pose_lin_setpoint_, pose_lin_measure_;
    Eigen::Quaterniond pose_ang_setpoint_, pose_ang_measure_inv_;
    // wrench command
    geometry_msgs::Wrench wrench_command_;

};

#endif // FREEFLOATINGBODYPID_H
