#include <freefloating_gazebo/hydro_link.h>
#include <Eigen/Geometry>

namespace ffg
{


Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d out;
  out << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return out;
}

Eigen::Vector3d HydroLink::buoyancyForce(double surface_distance)
{
  Eigen::Vector3d force(0,0,buoyancy_force);

  if(surface_distance > -buoyancy_limit)
  {
    if(surface_distance > buoyancy_limit)
      force *= 0;
    else
      force *= cos(M_PI/4.*(surface_distance/buoyancy_limit + 1));
  }
  return force;
}

Eigen::Vector6d HydroLink::hydroDynamicForce(Eigen::Vector6d &vel)
{
  Eigen::Vector6d force(Eigen::Vector6d::Zero());

  // filter velocity
  if(has_added_mass)
    vel_filter.filter(vel);

  // damping part
  if(has_lin_damping)
    force = -lin_damping.cwiseProduct(vel);
  if(has_quad_damping)
    force -= quad_damping.cwiseProduct(vel.cwiseProduct(vel.cwiseAbs()));

  // added mass part
  if(has_added_mass)
  {
    const Eigen::Vector6d acc = (vel - vel_prev)/dt;
    force -= added_mass * acc;

    // added Coriolis
    Eigen::Matrix6d Cor;
    const Eigen::Vector6d A = added_mass * vel;
    const Eigen::Matrix3d Sa = -1 * skew(A.head<3>());
    Cor << Eigen::Matrix3d::Zero(), Sa, Sa, skew(-A.tail<3>());
    force -= Cor * vel;

    vel_prev = vel;
  }
  return force;
}

}

