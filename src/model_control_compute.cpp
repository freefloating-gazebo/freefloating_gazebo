#include <freefloating_gazebo/model_control_compute.h>

namespace ffg
{

void ModelControlCompute::UpdateError()
{
    //Let's update the pose_error_
    Eigen::Quaterniond pose_ang_measure_ = pose_ang_measure_inv_.inverse();
    Eigen::Vector3d attitude_error_;

    attitude_error_ = pose_ang_measure_.w()*pose_ang_setpoint_.vec() - pose_ang_setpoint_.w()*pose_ang_measure_.vec() + pose_ang_setpoint_.vec().cross( pose_ang_measure_.vec() );
    pose_error_ << pose_ang_measure_inv_.toRotationMatrix() * (pose_lin_setpoint_ - pose_lin_measure_) , attitude_error_;

    //Let's update the velocity_error_
    velocity_error_ = velocity_setpoint_ - velocity_measure_;

    //Let's update the s_error_
    Eigen::DiagonalMatrix<double, 6> Lambda(lp, lp, lp, lo, lo, lo);
    s_error_ = velocity_error_ + Lambda * pose_error_;
}

void ModelControlCompute::UpdateWrench()
{
    //Let's Calculate the regressor matrix
    Eigen::DiagonalMatrix<double, 6> lin_dampling_regressor( velocity_measure_ );
}

void ModelControlCompute::UpdateParam()
{

}

}

