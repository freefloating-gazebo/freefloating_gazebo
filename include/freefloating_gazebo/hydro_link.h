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
    cog.setZero();
    cob.setZero();
    vel_prev.setZero();
    added_mass.setZero();
    inertia.setZero();
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

  Eigen::Vector3d cob, cog;
  Eigen::Vector6d vel_prev;
  double buoyancy_force = 0, buoyancy_limit = 0, mass = 0;

  Eigen::Matrix6d inertia;
  Eigen::Vector6d lin_damping, quad_damping;
  Eigen::Matrix6d added_mass;
  bool has_lin_damping = false;
  bool has_quad_damping = false;
  bool has_added_mass = false;

private:
  double dt;
  Butterworth_nD vel_filter;
};

}

#endif // HYDRO_LINK_H
