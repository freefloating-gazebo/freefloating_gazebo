#include <freefloating_gazebo/freefloating_pids.h>

using std::cout;
using std::endl;
using std::string;

FreeFloatingPids::FreeFloatingPids()
{
    position_pids_.clear();
    velocity_pids_.clear();
    axes_.clear();
}

void FreeFloatingPids::initSwitchServices(std::string name)
{
    services_.clear();
    services_.push_back(pid_node_.advertiseService<CTreq, CTres>(name + "_position_control",
                                                                 boost::bind(&FreeFloatingPids::ToPositionControl, this, _1, _2)));
    services_.push_back(pid_node_.advertiseService<CTreq, CTres>(name + "_velocity_control",
                                                                 boost::bind(&FreeFloatingPids::ToVelocityControl, this, _1, _2)));
    services_.push_back(pid_node_.advertiseService<CTreq, CTres>(name + "_effort_control",
                                                                 boost::bind(&FreeFloatingPids::ToEffortControl, this, _1, _2)));
}

void FreeFloatingPids::UpdatePositionPID()
{
    if(setpoint_position_ok_)
    {
        for(auto & pid: position_pids_)
            pid.updateCommand(dt_);
    }
}

void FreeFloatingPids::UpdateVelocityPID()
{
    if(setpoint_velocity_ok_)
    {
        for(auto & pid: velocity_pids_)
            pid.updateCommand(dt_);
    }
}

bool FreeFloatingPids::ToPositionControl(CTreq _req, CTres &_res)
{
    for(int i = 0; i < axes_.size(); ++i)
    {
        // to position control if in req.axes or empty req.axes
        if((_req.axes.size() == 0) ||
                (std::find(_req.axes.begin(), _req.axes.end(),
                           axes_[i]) != _req.axes.end()))
        {
            position_pids_[i].active = true;
            // activate velocity if cascaded position PID
            velocity_pids_[i].active = cascaded_position_;
        }
    }
    position_used_ = std::find_if(position_pids_.begin(), position_pids_.end(),
                                  [](const pid_st &pid){return pid.active;})
                              != position_pids_.end();
    printControlType();
    return true;
}



bool FreeFloatingPids::ToVelocityControl(CTreq _req, CTres &_res)
{
    for(int i = 0; i < axes_.size(); ++i)
    {
        // to velocity control if in req.axes or empty req.axes
        if((_req.axes.size() == 0) ||
                (std::find(_req.axes.begin(), _req.axes.end(),
                           axes_[i]) != _req.axes.end()))
        {
            velocity_pids_[i].active = true;
            position_pids_[i].active = false;
        }
    }
    velocity_used_ = std::find_if(velocity_pids_.begin(), velocity_pids_.end(),
                                  [](const pid_st &pid){return pid.active;})
                              != velocity_pids_.end();

    printControlType();
    return true;
}


bool FreeFloatingPids::ToEffortControl(CTreq _req, CTres &_res)
{
    for(int i = 0; i < axes_.size(); ++i)
    {
         // to effort control if in req.axes or empty req.axes
        if((_req.axes.size() == 0) ||
                (std::find(_req.axes.begin(), _req.axes.end(),
                           axes_[i]) != _req.axes.end()))
            position_pids_[i].active = velocity_pids_[i].active = false;
    }
    printControlType();
    return true;
}

void FreeFloatingPids::printControlType()
{
    std::cout << "Control modes:\n";
    for(int i = 0; i < axes_.size(); ++i)
    {
        std::cout << "   -  " << axes_[i] << ": ";
        if(position_pids_[i].active)
            std::cout << "position\n";
        else if(velocity_pids_[i].active)
            std::cout << "velocity\n";
        else
            std::cout << "effort\n";
    }
    std::cout << std::endl;
}



void FreeFloatingPids::InitPID(control_toolbox::Pid &_pid, const ros::NodeHandle&_node, const bool &_use_dynamic_reconfig)
{
    if(_use_dynamic_reconfig)
    {
        // write anti windup
        _node.setParam("antiwindup", true);
        // classical PID init
        _pid.init(_node);
    }
    else
    {
        control_toolbox::Pid::Gains gains;

        // Load PID gains from parameter server
        if (!_node.getParam("p", gains.p_gain_))
        {
            ROS_ERROR("No p gain specified for pid.  Namespace: %s", _node.getNamespace().c_str());
            return;
        }
        // Only the P gain is required, the I and D gains are optional and default to 0:
        _node.param("i", gains.i_gain_, 0.0);
        _node.param("d", gains.d_gain_, 0.0);

        // Load integral clamp from param server or default to 0
        double i_clamp;
        _node.param("i_clamp", i_clamp, 0.0);
        gains.i_max_ = std::abs(i_clamp);
        gains.i_min_ = -std::abs(i_clamp);
        _pid.setGains(gains);
    }
}
