
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
  robot_namespace_ = model_->GetName();
  controller_is_running_ = true;

  // register ROS node & time
  rosnode_ = ros::NodeHandle(robot_namespace_);
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
  mapper_.parse(_sdf->ToString(""), false);
  // add steering thrusters if any
  thruster_links_.clear();
  for(const auto &name: mapper_.names)
  {
    for(auto link: _model->GetLinks())
    {
      if(link->GetName() == name)
      {
        thruster_links_.push_back(link);
        break;
      }
    }
  }

  if(!mapper_.has_thrusters())
    ROS_INFO("No thrusters found in %s", _model->GetName().c_str());

  // *** SET UP BODY CONTROL
  char param[FILENAME_MAX];
  if(mapper_.has_thrusters())
  {
    // initialize subscriber to body commands
    thruster_command_.resize(static_cast<long>(mapper_.names.size()));
    // initialize publisher to thruster_use
    thruster_use_.name = mapper_.names;
    thruster_use_.position.resize(mapper_.names.size());
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
  ROS_INFO("Started FreeFloating Control Plugin for %s.", _model->GetName().c_str());
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
    if(control_body_ && body_command_received_ && (body_->GetWorldCoGPose().pos.z < z_surface_))
#else
    if(mapper_.has_thrusters() && body_command_received_ && (body_->WorldCoGPose().Pos().Z() < z_surface_))
#endif
    {
      mapper_.saturate(thruster_command_);

      // build and apply wrench for fixed thrusters
      if(mapper_.fixed_idx.size())
      {
        Eigen::VectorXd fixed(mapper_.fixed_idx.size());
        for(size_t i=0;i<mapper_.fixed_idx.size();++i)
          fixed(static_cast<Eigen::Index>(i)) = thruster_command_(static_cast<Eigen::Index>(mapper_.fixed_idx[i]));
        // to wrench
        fixed = mapper_.map * fixed;
        // apply this wrench to body
#ifdef GAZEBOLD
        body_->AddForceAtWorldPosition(body_->GetWorldPose().rot.RotateVector(Vector3d(fixed(0), fixed(1), fixed(2))), body_->GetWorldCoGPose().pos);
#else
        body_->AddForceAtWorldPosition(body_->WorldPose().Rot().RotateVector(Vector3d(fixed(0), fixed(1), fixed(2))), body_->WorldCoGPose().Pos());
#endif
        body_->AddRelativeTorque(Vector3d(fixed(3), fixed(4), fixed(5)));
      }

      // apply command for steering thrusters
      if(mapper_.steer_idx.size())
      {
        for(unsigned int i=0;i<mapper_.steer_idx.size();++i)
          thruster_links_[i]->AddRelativeForce(Vector3d(0,0,-thruster_command_(mapper_.steer_idx[i])));
      }

      // compute and publish thruster use in %
      for(size_t i=0;i<thruster_command_.size();++i)
        thruster_use_.position[i] = 100*std::abs(thruster_command_(i) / mapper_.max_command[i]);
      thruster_use_publisher_.publish(thruster_use_);
    }
  }

  // publish joint states anyway
  double t = ros::Time::now().toSec();
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
  if(!mapper_.has_thrusters())
    return;

  const bool read_effort = _msg->effort.size();

  if(read_effort && (_msg->name.size() != _msg->effort.size()))
  {
    ROS_WARN("Received inconsistent thruster command, name and effort dimension do not match");
    return;
  }
  else
    if(!read_effort && (_msg->name.size() != _msg->position.size()))
    {
      ROS_WARN("Received inconsistent thruster command, name and position dimension do not match");
      return;
    }

  body_command_received_ = true;
  // store thruster command
  for(unsigned int i=0;i<mapper_.names.size();++i)
  {
    for(unsigned int j=0;j<_msg->name.size();++j)
    {
      if(mapper_.names[i] == _msg->name[j])
        thruster_command_(i) = read_effort?_msg->effort[j]:_msg->position[j];
    }
  }
}




}   // namespace gazebo
