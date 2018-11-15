#include <freefloating_gazebo/freefloating_pids_body.h>

using std::cout;
using std::endl;
using std::string;


void FreeFloatingBodyPids::Init(ros::NodeHandle &nh, ros::Duration&_dt,
                                const std::vector<std::string>&_controlled_axes,
                                std::string default_mode)
{
  // init dt from rate
  dt = _dt;

  // wrench setpoint
  wrench_sp_subscriber =
      nh.subscribe("body_wrench_setpoint", 1, &FreeFloatingBodyPids::WrenchSPCallBack, this);
  // measure
  state_subscriber =
      nh.subscribe("state", 1, &FreeFloatingBodyPids::MeasureCallBack, this);

  // deal with controlled axes
  const size_t n = _controlled_axes.size();
  axes.resize(n);

  const std::vector<std::string> axes3D{"x", "y", "z", "roll", "pitch", "yaw"};

  // get whether or not we use dynamic reconfigure
  bool use_dynamic_reconfig;
  ros::NodeHandle control_node(nh, "controllers");
  control_node.param("controllers/config/body/dynamic_reconfigure", use_dynamic_reconfig, true);

  if(n)
  {
    // position setpoint
    position_sp_subscriber =
        nh.subscribe("body_position_setpoint", 1, &FreeFloatingBodyPids::PositionSPCallBack, this);
    // velocity setpoint
    velocity_sp_subscriber =
        nh.subscribe("body_velocity_setpoint", 1, &FreeFloatingBodyPids::VelocitySPCallBack, this);
  }

  for(unsigned int i=0;i<n;++i)
  {
    const auto idx = static_cast<size_t>(std::distance(axes3D.begin(),
                                                       std::find(axes3D.begin(),
                                                                 axes3D.end(),
                                                                 _controlled_axes[i])));
    auto axis = &axes[i];
    axis->name = _controlled_axes[i];
    // here we have the controlled axis
    switch(idx)
    {
    case 0:
      axis->position.error = &(pose_lin_error_.x());
      axis->velocity.error = &(velocity_lin_error_.x());
      axis->position.command = axis->velocity.command = &(wrench_command_.force.x);
      break;
    case 1:
      axis->position.error = &(pose_lin_error_.y());
      axis->velocity.error = &(velocity_lin_error_.y());
      axis->position.command = axis->velocity.command = &(wrench_command_.force.y);
      break;
    case 2:
      axis->position.error = &(pose_lin_error_.z());
      axis->velocity.error = &(velocity_lin_error_.z());
      axis->position.command = axis->velocity.command = &(wrench_command_.force.z);
      break;
    case 3:
      axis->position.error = &(pose_ang_error_.x());
      axis->velocity.error = &(velocity_ang_error_.x());
      axis->position.command = axis->velocity.command = &(wrench_command_.torque.x);
      break;
    case 4:
      axis->position.error = &(pose_ang_error_.y());
      axis->velocity.error = &(velocity_ang_error_.y());
      axis->position.command = axis->velocity.command = &(wrench_command_.torque.y);
      break;
    case 5:
      axis->position.error = &(pose_ang_error_.z());
      axis->velocity.error = &(velocity_ang_error_.z());
      axis->position.command = axis->velocity.command = &(wrench_command_.torque.z);
      break;
    }
    InitPID(axis->position.pid, ros::NodeHandle(control_node, axis->name + "/position"), use_dynamic_reconfig);
    InitPID(axis->velocity.pid, ros::NodeHandle(control_node, axis->name + "/velocity"), use_dynamic_reconfig);
  }

  // default control = position
  CTreq req;
  CTres res;
  if(n)
    ToPositionControl(req, res);
  else
    ToEffortControl(req, res);
  if(default_mode == "velocity")
  {
    ToVelocityControl(req, res);
  }
  else if(default_mode == "depth")
  {
    req.axes = {"x", "y", "yaw"};
    ToVelocityControl(req, res);
  }
  else if(default_mode == "effort")
  {
    req.axes = {"x", "y", "z", "yaw"};
    ToEffortControl(req, res);
  }
  initSwitchServices(control_node, "body");
}


bool FreeFloatingBodyPids::UpdatePID()
{
  bool updated = false;
  if(state_received)
  {
    if(setpoint_position_ok && position_used)    // setpoint & need for position PID
    {
      Eigen::Matrix3d world_to_body = pose_ang_measure_inv_.toRotationMatrix();
      // express pose error in the body frame
      pose_lin_error_ = world_to_body * (pose_lin_setpoint_ - pose_lin_measure_);
      // quaternion error in the body frame

      Eigen::Quaterniond q(pose_ang_setpoint_ * pose_ang_measure_inv_);
      const double sin_theta_over_2 = sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
      if(std::abs(sin_theta_over_2) < 1e-6)
        pose_ang_error_ = Eigen::Vector3d(0,0,0);
      else
      {
        Eigen::Vector3d u(q.x(),q.y(),q.z());
        u *= 1./sin_theta_over_2;
        pose_ang_error_ = 2*atan2(sin_theta_over_2, q.w()) * world_to_body * u;
      }
      UpdatePositionPID();
      updated = true;
    }

    if(setpoint_velocity_ok && velocity_used)
    {
      // velocity error is already in the body frame
      velocity_lin_error_ =  velocity_lin_setpoint_ - velocity_lin_measure_;
      velocity_ang_error_ =  velocity_ang_setpoint_ - velocity_ang_measure_;

      //cout << "Velocity lin error in WF: " << (velocity_lin_error_).transpose() << endl;
      // writes the wrench command
      UpdateVelocityPID();
      updated = true;
    }
  }
  return updated;
}


// parse received position setpoint
void FreeFloatingBodyPids::PositionSPCallBack(const geometry_msgs::PoseStampedConstPtr& _msg)
{
  setpoint_position_ok = true;

  pose_lin_setpoint_ = Eigen::Vector3d(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
  pose_ang_setpoint_ = Eigen::Quaterniond(_msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z);
}

// parse received velocity setpoint
void FreeFloatingBodyPids::VelocitySPCallBack(const geometry_msgs::TwistStampedConstPtr & _msg)
{
  setpoint_velocity_ok = true;
  velocity_lin_setpoint_ = Eigen::Vector3d(_msg->twist.linear.x, _msg->twist.linear.y, _msg->twist.linear.z);
  velocity_ang_setpoint_ = Eigen::Vector3d(_msg->twist.angular.x, _msg->twist.angular.y, _msg->twist.angular.z);
}


void FreeFloatingBodyPids::MeasureCallBack(const nav_msgs::OdometryConstPtr &_msg)
{
  state_received = true;
  // positions are expressed in the world frame, rotation is inversed
  pose_lin_measure_ = Eigen::Vector3d(_msg->pose.pose.position.x, _msg->pose.pose.position.y, _msg->pose.pose.position.z);
  pose_ang_measure_inv_ = Eigen::Quaterniond(_msg->pose.pose.orientation.w, _msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z).inverse();

  // change velocities from world to body frame
  velocity_lin_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.linear.x, _msg->twist.twist.linear.y, _msg->twist.twist.linear.z);
  velocity_ang_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.angular.x, _msg->twist.twist.angular.y, _msg->twist.twist.angular.z);
}
