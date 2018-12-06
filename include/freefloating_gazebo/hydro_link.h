#ifndef HYDRO_LINK_H
#define HYDRO_LINK_H

#include <string>
#include <Eigen/Core>
#include <freefloating_gazebo/butterworth.h>

namespace Eigen
{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}


namespace ffg
{

Eigen::Matrix3d skew(const Eigen::Vector3d &v);


class HydroLink
{
public:
  HydroLink()
  {
    buoyancy_center.setZero();
    vel_prev.setZero();
    added_mass.setZero();
    lin_damping.setZero();
    quad_damping.setZero();
  }

  void initFilters(double _dt, double frequency = 10)
  {
    dt = _dt;
    vel_filter = Butterworth_nD(6, frequency, dt);
  }

  Eigen::Vector3d buoyancyForce(double surface_distance);
  Eigen::Vector6d hydroDynamicForce(Eigen::Vector6d &vel);

  double dt;
  Butterworth_nD vel_filter;
  std::string name;
  Eigen::Vector3d buoyancy_center;
  Eigen::Vector6d vel_prev;
  double buoyancy_force = 0, buoyancy_limit = 0;

  Eigen::Vector6d lin_damping, quad_damping;
  Eigen::Matrix6d added_mass;
  bool has_lin_damping = false;
  bool has_quad_damping = false;
  bool has_added_mass = false;


};

}

#endif // HYDRO_LINK_H
