#include <freefloating_gazebo/freefloating_pids_joint.h>
#include <urdf/model.h>

using std::cout;
using std::endl;
using std::string;


void FreeFloatingJointPids::Init(ros::NodeHandle &nh,
                                 const ros::Duration &_dt,
                                 std::string default_mode)
{
  // init dt from rate
  dt = _dt;

  ros::NodeHandle control_node (nh, "controllers");

  // setpoint
  setpoint_subscriber = nh.subscribe("joint_setpoint", 1, &FreeFloatingJointPids::SetpointCallBack, this);
  // measure
  state_subscriber = nh.subscribe("joint_states", 1, &FreeFloatingJointPids::MeasureCallBack, this);
  // command
  command_publisher = nh.advertise<sensor_msgs::JointState>("joint_command", 1);

  // filter param
  alpha = .5;

  // get params from server

  control_node.getParam("config/joints/name", joint_setpoint.name);

  // get whether or not we use a cascaded controller for position
  control_node.param("config/joints/cascaded_position", cascaded_position, true);

  // get whether or not we use dynamic reconfigure
  bool use_dynamic_reconfig;
  control_node.param("config/joints/dynamic_reconfigure", use_dynamic_reconfig, true);

  // resize vectors
  const auto n = joint_setpoint.name.size();
  position_error.resize(n);
  velocity_error.resize(n);
  position_filtered_measure.resize(n);
  velocity_filtered_measure.resize(n);

  velocity_measure.resize(n);
  position_measure.resize(n);
  joint_setpoint.velocity.resize(n);
  joint_setpoint.position.resize(n);
  joint_command.name = joint_setpoint.name;
  joint_command.effort.resize(n);

  // store limits
  control_node.getParam("config/joints/upper", joint_upper);
  control_node.getParam("config/joints/lower", joint_lower);
  control_node.getParam("config/joints/velocity", joint_max_velocity);
  vmax_is_set = !cascaded_position;

  axes.resize(n);
  for(size_t i=0;i<n;++i)
  {
    auto axis = &axes[i];
    axis->name = joint_command.name[i];
    axis->position.error = &(position_error[i]);
    axis->velocity.error = &(velocity_error[i]);
    axis->velocity.command = &(joint_command.effort[i]);

    // position PID output depends on control type
    if(cascaded_position)
      axis->position.command = &(joint_setpoint.velocity[i]);
    else
      axis->position.command = &(joint_command.effort[i]);

    position_filtered_measure[i] = velocity_filtered_measure[i] = 0;

    InitPID(axis->position.pid, ros::NodeHandle(control_node, axis->name + "/position"), use_dynamic_reconfig);
    InitPID(axis->velocity.pid, ros::NodeHandle(control_node, axis->name + "/velocity"), use_dynamic_reconfig);
  }
  initSwitchServices(control_node, "joints");

  // init to default control
  CTreq req;
  CTres res;

  if(default_mode == "velocity")
    ToVelocityControl(req, res);
  else if(default_mode == "effort")
    ToEffortControl(req, res);
  else
    ToPositionControl(req, res);
}


bool FreeFloatingJointPids::UpdatePID()
{
  bool updated = false;
  if(!vmax_is_set)
  {
    for(size_t i=0;i<velocity_error.size();++i)
      joint_max_velocity[i] = axes[i].position.pid.getGains().i_max_;
    vmax_is_set = true;
  }

  if(state_received)
  {
    // do position PID computations only if needed
    if(setpoint_position_ok && position_used)
    {
      //  cout << "Joint position error: ";
      // get position error
      for(unsigned int i=0;i<position_error.size();++i)
      {
        position_filtered_measure[i] = filters::exponentialSmoothing(position_measure[i], position_filtered_measure[i], alpha);
        position_error[i] = filters::clamp(joint_setpoint.position[i], joint_lower[i], joint_upper[i]) - position_filtered_measure[i];
        //      cout << position_error_[i];
      }
      //   cout << endl;
      // update pid's
      UpdatePositionPID();
      updated = true;

      // has written new velocity setpoint
    }

    if(setpoint_velocity_ok && velocity_used)
    {
      // get velocity error
      for(unsigned int i=0;i<velocity_error.size();++i)
      {
        // if joint is a max bound, error is 0
        if((position_measure[i] >= joint_upper[i] && joint_setpoint.velocity[i] > 0) ||
           (position_measure[i] <= joint_lower[i] && joint_setpoint.velocity[i] < 0))
          velocity_error[i] = 0;
        else
        {
          velocity_filtered_measure[i] = filters::exponentialSmoothing(velocity_measure[i], velocity_filtered_measure[i], alpha);
          //  cout << "Joint " << i+1 << "vel: " << joint_setpoint_.velocity[i] << ", min: " << -joint_max_velocity_[i] << ", max: " << joint_max_velocity_[i] << endl;
          velocity_error[i] = filters::clamp(joint_setpoint.velocity[i], -joint_max_velocity[i], joint_max_velocity[i]) - velocity_filtered_measure[i];
        }
      }

      // update pid's
      UpdateVelocityPID();
      updated = true;
    }
  }
  return updated;
}

void FreeFloatingJointPids::SetpointCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
  // check for joint ordering
  for(size_t i=0;i<joint_setpoint.name.size();++i)
  {
    // get corresponding index in message
    const auto idx = static_cast<size_t>(std::distance(_msg->name.begin(),
                                                       std::find(_msg->name.begin(),
                                                                 _msg->name.end(),
                                                                 joint_setpoint.name[i])));
    if(idx < _msg->name.size())
    {
      if(_msg->position.size() > idx)
      {
        joint_setpoint.position[i] = _msg->position[idx];
        setpoint_position_ok = true;
      }
      if(_msg->velocity.size() > idx)
      {
        joint_setpoint.velocity[i] = _msg->velocity[idx];
        setpoint_velocity_ok = true;
      }
      if(_msg->effort.size() > idx)
        joint_command.effort[i] = _msg->effort[idx];
    }
  }
}

void FreeFloatingJointPids::MeasureCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
  state_received = true;
  // check for joint ordering
  for(size_t i=0;i<joint_setpoint.name.size();++i)
  {
    // get corresponding index in message
    const auto idx = static_cast<size_t>(std::distance(_msg->name.begin(),
                                                       std::find(_msg->name.begin(),
                                                                 _msg->name.end(),
                                                                 joint_setpoint.name[i])));
    if(idx < _msg->name.size())
    {
      if(_msg->position.size() > idx)
        position_measure[i] = _msg->position[idx];
      if(_msg->velocity.size() > idx)
        velocity_measure[i] = _msg->velocity[idx];
    }
  }
}

bool FreeFloatingJointPids::writeJointLimits(ros::NodeHandle & nh)
{
  urdf::Model model;
  model.initParam("robot_description");
  ros::NodeHandle control_node(nh, "controllers");

  bool cascaded_position = true;
  if(control_node.hasParam("config/joints/cascaded_position"))
    control_node.getParam("config/joints/cascaded_position", cascaded_position);

  std::vector<std::string> joint_names;
  std::vector<double> joint_min, joint_max, vel_max;
  for(const auto &joint: model.joints_)
  {
    const std::string name = joint.first;

    if(control_node.hasParam(name))
    {
      const auto joint_ptr = joint.second;
      if(joint_ptr->type != urdf::Joint::FIXED)
      {

        // set max velocity or max effort for the position PID
        if(cascaded_position)
          control_node.setParam(name + "/position/i_clamp",
                                joint_ptr->limits->velocity);
        else
          control_node.setParam(name + "/position/i_clamp",
                                joint_ptr->limits->effort);


        // set max effort for the velocity PID
        control_node.setParam(name + "/velocity/i_clamp",
                              joint_ptr->limits->effort);

        // set antiwindup to true - why would anyone set it to false?
        control_node.setParam(name + "/position/antiwindup", true);
        control_node.setParam(name + "/velocity/antiwindup", true);

        // save name and joint limits
        joint_names.push_back(name);
        joint_min.push_back(joint_ptr->limits->lower);
        joint_max.push_back(joint_ptr->limits->upper);
        vel_max.push_back(joint_ptr->limits->velocity);
      }
    }

  }

  // push setpoint topic, name, lower and bound
  if(joint_names.size())
  {
    control_node.setParam("config/joints/name", joint_names);
    control_node.setParam("config/joints/lower", joint_min);
    control_node.setParam("config/joints/upper", joint_max);
    control_node.setParam("config/joints/velocity", vel_max);
    return true;
  }
  return false;
}
