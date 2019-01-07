#ifndef THRUSTER_ALLOCATOR_H
#define THRUSTER_ALLOCATOR_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <freefloating_gazebo/hydro_model_parser.h>

namespace ffg
{

class ThrusterAllocator
{
public:
  // parse raw param to get thruster max force and map
  ThrusterAllocator(ros::NodeHandle &nh);

  std::vector<std::string> initControl(ros::NodeHandle &nh, double map_threshold = 1e-2);
  bool has_thrusters() const {return names.size();}

  geometry_msgs::Wrench maxWrench() const
  {
    geometry_msgs::Wrench wrench;
    wrench.force.x = max_wrench[0];
    wrench.force.y = max_wrench[1];
    wrench.force.z = max_wrench[2];
    wrench.torque.x = max_wrench[3];
    wrench.torque.y = max_wrench[4];
    wrench.torque.z = max_wrench[5];
    return wrench;
  }

  void saturate(Eigen::VectorXd &_command) const;

  sensor_msgs::JointState wrench2Thrusters(const geometry_msgs::Wrench  & cmd) const;

  ffg::HydroLink base_link;
  std::vector<uint> steer_idx, fixed_idx;
  std::vector<std::string> names;
  std::vector<double> max_thrust, max_wrench, max_vel;

  Eigen::MatrixXd map;
  Eigen::MatrixXd inverse_map;
};

}



#endif // THRUSTER_ALLOCATOR_H
