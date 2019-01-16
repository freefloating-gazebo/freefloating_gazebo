
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <freefloating_gazebo/freefloating_gazebo_control.h>
#include <freefloating_gazebo/hydro_model_parser.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <functional>

using std::cout;
using std::endl;
using std::string;
using ignition::math::Vector3d;

#if GAZEBO_MAJOR_VERSION < 9
#define GAZEBOLD
ignition::math::Vector3d v3convert(gazebo::math::Vector3 src)
{
  return ignition::math::Vector3d(src.x, src.y, src.z);
}
#endif


template <typename T>
void clamp(T& in, T low, T high)
{
  if(in < low)
    in = low;
  else if(in > high)
    in = high;
}

namespace gazebo
{

bool FreeFloatingControlPlugin::SwitchService(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  if(controller_is_running_)
    ROS_INFO("Switching freefloating_control OFF");
  else
    ROS_INFO("Switching freefloating_control ON");
  controller_is_running_ = !controller_is_running_;
  return true;
}

void FreeFloatingControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // get model and name
  model_ = _model;
  const auto robot_name = model_->GetName();
  controller_is_running_ = true;

  // register ROS node & time
  rosnode_ = ros::NodeHandle(robot_name);
  ros::NodeHandle control_node(rosnode_, "controllers");
  t_prev_ = 0;

  // get surface Z
  rosnode_.getParam("/gazebo/surface", z_surface_);

  // look for thrusters
  if(_sdf->HasElement("link"))
    body_ = model_->GetLink(_sdf->Get<std::string>("link"));
  else
    body_ = model_->GetLinks()[0];

  // parse thruster elements
  ffg::HydroModelParser parser;
  parser.parseThrusters(_sdf->ToString(""), robot_name);
  const auto n_thr = parser.thrusterInfo(thruster_fixed,
                                            thruster_steering,
                                            thruster_use_.name,
                                            thruster_max);
  // find link of steering thrusters
  auto idx = thruster_steering.begin();
  thruster_links_.clear();
  while(idx != thruster_steering.end())
  {
    auto link = _model->GetLink(thruster_use_.name[*idx]);
    if(link != nullptr)
    {
      thruster_links_.push_back(link);
      idx++;
    }
    else
    {
      ROS_WARN("%s: thruster %s referenced but link not found",
               _model->GetName().c_str(),
               thruster_use_.name[*idx].c_str());
      idx = thruster_steering.erase(idx);
    }
  }

  if(!n_thr)
    ROS_INFO("%s: no thrusters found", _model->GetName().c_str());

  // *** SET UP BODY CONTROL
  if(n_thr)
  {
    thruster_map = parser.thrusterMap();
    // initialize subscriber to body commands
    thruster_command_.resize(static_cast<long>(n_thr));
    // initialize publisher to thruster_use
    thruster_use_.position.resize(n_thr);
    thruster_use_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("thruster_use", 1);

    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "thruster_command", 1,
          boost::bind(&FreeFloatingControlPlugin::ThrusterCommandCallBack, this, _1),
          ros::VoidPtr(), &callback_queue_);

    body_command_subscriber_ = rosnode_.subscribe(ops);
    body_command_received_ = false;
  }
  // *** END BODY CONTROL

  // *** JOINT CONTROL
  if(model_->GetJointCount() != 0)
  {
    // initialize subscriber to joint commands
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "joint_command", 1,
          boost::bind(&FreeFloatingControlPlugin::JointCommandCallBack, this, _1),
          ros::VoidPtr(), &callback_queue_);
    joint_command_subscriber_ = rosnode_.subscribe(ops);
    joint_command_received_ = false;

    for(const auto &joint: model_->GetJoints())
      joint_states_.name.push_back(joint->GetName());

    // setup joint_states publisher
    joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("joint_states", 1);
    joint_states_.position.resize(model_->GetJointCount());
    joint_states_.velocity.resize(model_->GetJointCount());
  }
  // *** END JOINT CONTROL

  // store update rate
  if(_sdf->HasElement("updateRate"))
    update_T_ = 1./_sdf->Get<double>("updateRate");
  else
    update_T_ = 0;

  // Register plugin update
  update_event_ = event::Events::ConnectWorldUpdateBegin(std::bind(&FreeFloatingControlPlugin::Update, this));

  ros::spinOnce();
  ROS_INFO("%s: Started FreeFloating Control Plugin.", _model->GetName().c_str());
}

void FreeFloatingControlPlugin::Update()
{
  // activate callbacks
  callback_queue_.callAvailable();

  if(controller_is_running_)
  {
    // deal with joint control
    if(joint_command_received_)
    {
      for(size_t i=0;i<joint_command_.name.size();++i)
      {
        // find corresponding model joint
        const auto idx = static_cast<size_t>(std::distance(joint_states_.name.begin(),
                                                           std::find(joint_states_.name.begin(),
                                                                     joint_states_.name.end(),
                                                                     joint_command_.name[i])));
        if(idx != model_->GetJointCount())
          model_->GetJoints()[idx]->SetForce(0,joint_command_.effort[i]);
      }
    }

    // deal with body control if underwater
#ifdef GAZEBOLD
    if(thruster_max.size() && body_command_received_ && (body_->GetWorldCoGPose().pos.z < z_surface_))
#else
    if(thruster_max.size() && body_command_received_ && (body_->WorldCoGPose().Pos().Z() < z_surface_))
#endif
    {
      for(uint i = 0; i < thruster_command_.size(); ++i)
        clamp(thruster_command_[i], -thruster_max[i], thruster_max[i]);

      // build and apply wrench for fixed thrusters
      if(thruster_fixed.size())
      {
        Eigen::VectorXd fixed(thruster_fixed.size());
        for(uint i=0;i<thruster_fixed.size();++i)
          fixed[i] = thruster_command_[thruster_fixed[i]];
        // to wrench
        fixed = thruster_map * fixed;
        // apply this wrench to body
#ifdef GAZEBOLD
        body_->AddForceAtWorldPosition(body_->GetWorldPose().rot.RotateVector(Vector3d(fixed(0), fixed(1), fixed(2))), body_->GetWorldCoGPose().pos);
#else
        body_->AddForceAtWorldPosition(body_->WorldPose().Rot().RotateVector(Vector3d(fixed(0), fixed(1), fixed(2))), body_->WorldCoGPose().Pos());
#endif
        body_->AddRelativeTorque(Vector3d(fixed(3), fixed(4), fixed(5)));
      }

      // apply command for steering thrusters
      if(thruster_steering.size())
      {
        for(unsigned int i=0;i<thruster_steering.size();++i)
          thruster_links_[i]->AddRelativeForce(Vector3d(0,0,-thruster_command_(thruster_steering[i])));
      }

      // compute and publish thruster use in %
      for(size_t i=0;i<thruster_command_.size();++i)
        thruster_use_.position[i] = 100*std::abs(thruster_command_(i) / thruster_max[i]);
      thruster_use_publisher_.publish(thruster_use_);
    }
  }

  // publish joint states anyway
  const double t = ros::Time::now().toSec();
  if((t-t_prev_) > update_T_ && model_->GetJointCount())
  {
    t_prev_ = t;
    joint_states_.header.stamp = ros::Time::now();

    for(unsigned int i=0;i<model_->GetJointCount();++i)
    {
#ifdef GAZEBOLD
      joint_states_.position[i] = model_->GetJoints()[i]->GetAngle(0).Radian();
#else
      joint_states_.position[i] = model_->GetJoints()[i]->Position();
#endif
      joint_states_.velocity[i] = model_->GetJoints()[i]->GetVelocity(0);
    }
    joint_state_publisher_.publish(joint_states_);
  }
}

void FreeFloatingControlPlugin::ThrusterCommandCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
  if(!thruster_max.size())
    return;

  const bool read_effort = _msg->effort.size();

  if(read_effort && (_msg->name.size() != _msg->effort.size()))
  {
    ROS_WARN("%s: Received inconsistent thruster command, name and effort dimension do not match",
             model_->GetName().c_str());
    return;
  }
  else if(!read_effort && (_msg->name.size() != _msg->position.size()))
  {
    ROS_WARN("%s: Received inconsistent thruster command, name and position dimension do not match",
             model_->GetName().c_str());
    return;
  }

  body_command_received_ = true;
  // store thruster command
  for(unsigned int i=0;i<thruster_use_.name.size();++i)
  {
    for(unsigned int j=0;j<_msg->name.size();++j)
    {
      if(thruster_use_.name[i] == _msg->name[j])
        thruster_command_(i) = read_effort?_msg->effort[j]:_msg->position[j];
    }
  }
}

}   // namespace gazebo
