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
        double* error_ptr;
        double* command_ptr;
        control_toolbox::Pid pid;        
        bool active;
        void updateCommand(const ros::Duration &dt)
        {
            if(active)
                *command_ptr = pid.computeCommand(*error_ptr, dt);
        }
    };

public:
    FreeFloatingPids();

    // these updates do no check about the pointers
    void UpdatePositionPID();
    void UpdateVelocityPID();

    // service to switch controllers
    bool ToPositionControl(CTreq _req, CTres &_res);
    bool ToVelocityControl(CTreq _req, CTres &_res);
    bool ToEffortControl(CTreq _req, CTres &_res);
    void printControlType();

    // init services base on prefix
    void initSwitchServices(std::string name);

protected:

    void InitPID(control_toolbox::Pid& _pid, const ros::NodeHandle&_node, const bool &_use_dynamic_reconfig);

    // Basic PID's update: compute command from error, needed customized
    virtual bool UpdatePID() = 0;

protected:
    ros::NodeHandle pid_node_;
    std::vector<ros::ServiceServer> services_;
    ros::Duration dt_;
    std::vector<pid_st> position_pids_;
    std::vector<pid_st> velocity_pids_;
    std::vector<std::string> axes_;
    double error_filter;
    bool    setpoint_position_ok_ = false,
            setpoint_velocity_ok_ = false,
            state_received_ = false,
            cascaded_position_ = false,
            position_used_ = false,
            velocity_used_ = false;
};

#endif // FREEFLOATINGPID_H
