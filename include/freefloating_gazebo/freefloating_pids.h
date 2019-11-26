#ifndef FREEFLOATINGPID_H
#define FREEFLOATINGPID_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <control_toolbox/filters.h>
#include <ros/duration.h>
#include <ros/service_server.h>
#include <freefloating_gazebo/ControlType.h>
#include <freefloating_gazebo/butterworth.h>

class FreeFloatingPids
{
protected:

  typedef freefloating_gazebo::ControlTypeRequest CTreq;
  typedef freefloating_gazebo::ControlTypeResponse CTres;

  struct pid_st
  {
    double* error;
    double* command;
    control_toolbox::Pid pid;
    bool active = false;
    void updateCommand(const ros::Duration &dt)
    {
      *command = pid.computeCommand(*error, dt);
    }
  };

  struct Axis
  {
    pid_st position, velocity;
    std::string name;
  };

public:
  FreeFloatingPids();

  // these updates do no check about the pointers
  void UpdatePositionPID();
  void UpdateVelocityPID();

  bool ToPositionControl(const CTreq &_req, CTres &_res);
  bool ToVelocityControl(const CTreq &_req, CTres &_res);
  bool ToEffortControl(const CTreq &_req, CTres &_res);
  void checkControlTypes(const CTreq &_req);


  void initPositionControl(std::vector<std::string> _axes = {})
  {
    CTreq req;
    req.axes = _axes;
    CTres res;
    ToPositionControl(req, res);
  }
  void initVelocityControl(std::vector<std::string> _axes = {})
  {
    CTreq req;
    req.axes = _axes;
    CTres res;
    ToVelocityControl(req, res);
  }
  void initEffortControl(std::vector<std::string> _axes = {})
  {
    CTreq req;
    req.axes = _axes;
    CTres res;
    ToEffortControl(req, res);
  }

  // service to switch controllers
  void initSwitchServices(ros::NodeHandle &control_node, std::string name);

protected:

  void InitPID(control_toolbox::Pid& _pid, const ros::NodeHandle &pid_node, const bool &_use_dynamic_reconfig);

  // Basic PID's update: compute command from error, needed customized
  virtual bool UpdatePID() = 0;

protected:
  std::vector<ros::ServiceServer> services;
  ros::Duration dt;
  std::vector<Axis> axes;
  double error_filter;
  bool    setpoint_position_ok = false,
  setpoint_velocity_ok = false,
  state_received = false,
  cascaded_position = false,

  position_used = false,
  velocity_used = false;
};

#endif // FREEFLOATINGPID_H
