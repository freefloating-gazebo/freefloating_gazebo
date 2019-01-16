#include <freefloating_gazebo/freefloating_pids.h>

using std::cout;
using std::endl;
using std::string;

template<class T> void ignore( const T& ) { }

FreeFloatingPids::FreeFloatingPids()
{
  axes.clear();
}

void FreeFloatingPids::initSwitchServices(ros::NodeHandle &control_node, std::string name)
{
  services.clear();
  services.push_back(control_node.advertiseService<CTreq, CTres>(name + "_position_control",
                                                                 boost::bind(&FreeFloatingPids::ToPositionControl, this, _1, _2)));
  services.push_back(control_node.advertiseService<CTreq, CTres>(name + "_velocity_control",
                                                                 boost::bind(&FreeFloatingPids::ToVelocityControl, this, _1, _2)));
  services.push_back(control_node.advertiseService<CTreq, CTres>(name + "_effort_control",
                                                                 boost::bind(&FreeFloatingPids::ToEffortControl, this, _1, _2)));
}

void FreeFloatingPids::UpdatePositionPID()
{
    for(auto & axis: axes)
      if(axis.position.active)
        axis.position.updateCommand(dt);
}

void FreeFloatingPids::UpdateVelocityPID()
{
    for(auto & axis: axes)
      if(axis.velocity.active)
        axis.velocity.updateCommand(dt);
}

bool FreeFloatingPids::ToPositionControl(CTreq _req, CTres &_res)
{
  ignore(_res);
  for(auto & axis: axes)
  {
    // to position control if in req.axes or empty req.axes
    if((_req.axes.size() == 0) ||
       (std::find(_req.axes.begin(), _req.axes.end(),
                  axis.name) != _req.axes.end()))
    {
      axis.position.active = true;
      // activate velocity if cascaded position PID
      axis.velocity.active = cascaded_position;
    }
  }
  position_used = std::find_if(axes.begin(), axes.end(),
                               [](const Axis &axis){return axis.position.active;})
      != axes.end();
  printControlType();
  return true;
}


bool FreeFloatingPids::ToVelocityControl(CTreq _req, CTres &_res)
{
  ignore(_res);
  for(auto & axis: axes)
  {
    // to velocity control if in req.axes or empty req.axes
    if((_req.axes.size() == 0) ||
       (std::find(_req.axes.begin(), _req.axes.end(),
                  axis.name) != _req.axes.end()))
    {
      axis.velocity.active = true;
      axis.position.active = false;
    }
  }
  velocity_used = std::find_if(axes.begin(), axes.end(),
                               [](const Axis &axis){return axis.velocity.active;})
      != axes.end();

  printControlType();
  return true;
}


bool FreeFloatingPids::ToEffortControl(CTreq _req, CTres &_res)
{
  ignore(_res);
  for(auto & axis: axes)
  {
    // to effort control if in req.axes or empty req.axes
    if((_req.axes.size() == 0) ||
       (std::find(_req.axes.begin(), _req.axes.end(),
                  axis.name) != _req.axes.end()))
      axis.position.active = axis.velocity.active = false;
  }
  printControlType();
  return true;
}

void FreeFloatingPids::printControlType() const
{
  std::cout << "Control modes:\n";
  for(const auto & axis: axes)
  {
    std::cout << "   -  " << axis.name << ": ";
    if(axis.position.active)
      std::cout << "position\n";
    else if(axis.velocity.active)
      std::cout << "velocity\n";
    else
      std::cout << "effort\n";
  }
  std::cout << std::endl;
}

void FreeFloatingPids::InitPID(control_toolbox::Pid &_pid, const ros::NodeHandle &pid_node, const bool &_use_dynamic_reconfig)
{
  if(_use_dynamic_reconfig)
  {
    // write anti windup
    pid_node.setParam("antiwindup", true);
    // classical PID init
    _pid.init(pid_node);
  }
  else
  {
    control_toolbox::Pid::Gains gains;

    // Load PID gains from parameter server
    if (!pid_node.getParam("p", gains.p_gain_))
    {
      ROS_ERROR("No p gain specified for pid.  Namespace: %s", pid_node.getNamespace().c_str());
      return;
    }
    // Only the P gain is required, the I and D gains are optional and default to 0:
    pid_node.param("i", gains.i_gain_, 0.0);
    pid_node.param("d", gains.d_gain_, 0.0);

    // Load integral clamp from param server or default to 0
    double i_clamp;
    pid_node.param("i_clamp", i_clamp, 0.0);
    gains.i_max_ = std::abs(i_clamp);
    gains.i_min_ = -std::abs(i_clamp);
    _pid.setGains(gains);
  }
}

