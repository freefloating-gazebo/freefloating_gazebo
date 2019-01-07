#include <freefloating_gazebo/thruster_allocator.h>

namespace ffg
{

// parse from robot_description param
ThrusterAllocator::ThrusterAllocator(ros::NodeHandle &nh)
{
  HydroModelParser parser;
  parser.parseAll(nh);
  // store link
  base_link = parser.getLinks().find("base_link")->second;

  // keep dynamics
  parser.thrusterInfo(fixed_idx, steer_idx, names, max_thrust);
  max_wrench = parser.maxWrench();
  max_vel = parser.maxVelocity();
  map = parser.thrusterMap();
}

std::vector<std::string> ThrusterAllocator::initControl(ros::NodeHandle &nh, double map_threshold)
{
  std::vector<std::string> controlled_axes;
  const std::vector<std::string> axes{"x", "y", "z", "roll", "pitch", "yaw"};

  for(size_t axis = 0; axis < 6; axis++)
  {
    if(max_wrench[axis] > 1e-6)
    {
      char param[FILENAME_MAX];
      sprintf(param, "controllers/%s", axes[axis].c_str());
      if(nh.hasParam(param))
      {
        controlled_axes.push_back(axes[axis]);
        // push to position PID
        sprintf(param, "controllers/%s/position/i_clamp", axes[axis].c_str());
        nh.setParam(param, max_wrench[axis]);
        // push to velocity PID
        sprintf(param, "controllers/%s/velocity/i_clamp", axes[axis].c_str());
        nh.setParam(param, max_wrench[axis]);
      }
    }
  }

  // threshold on map before pseudo-inverse
  for(int r = 0; r < 6; ++r)
  {
    for(int c = 0; c < map.cols(); ++c)
    {
      if(std::abs(map(r,c)) < map_threshold)
        map(r,c) = 0;
    }
  }

  inverse_map.resize(map.cols(), map.rows());
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_M(map, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd dummy_in(map.rows());
  Eigen::VectorXd dummy_out(map.cols());
  unsigned int i,j;
  for(i=0;i<map.rows();++i)
  {
    dummy_in.setZero();
    dummy_in(i) = 1;
    dummy_out = svd_M.solve(dummy_in);
    for(j = 0; j<map.cols();++j)
      inverse_map(j,i) = dummy_out(j);
  }
  return controlled_axes;
}

void ThrusterAllocator::saturate(Eigen::VectorXd &_command) const
{
  double norm_ratio = 1;
  unsigned int i;
  for(i=0;i<fixed_idx.size();++i)
    norm_ratio = std::max(norm_ratio, std::abs(_command(i)) / max_thrust[i]);
  _command *= 1./norm_ratio;
}

sensor_msgs::JointState ThrusterAllocator::wrench2Thrusters(const geometry_msgs::Wrench &cmd) const
{
  Eigen::VectorXd wrench(6);
  wrench << cmd.force.x, cmd.force.y, cmd.force.z,
      cmd.torque.x, cmd.torque.y, cmd.torque.z;
  sensor_msgs::JointState msg;
  msg.name = names;
  msg.effort.reserve(names.size());

  Eigen::VectorXd thrust = inverse_map * wrench;
  saturate(thrust);

  for(int i = 0; i < thrust.rows(); i++)
    msg.effort.push_back(thrust[i]);
  return msg;
}
}
