#include <freefloating_gazebo/model_control_compute.h>
#include <freefloating_gazebo/hydro_link.h>

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
    Eigen::DiagonalMatrix<double, 6> Lambda;
    Lambda.diagonal() << lp, lp, lp, lo, lo, lo;
    s_error_ = velocity_error_ + Lambda * pose_error_;

}

void ModelControlCompute::UpdateParam()
{
     param_prev = param_estimated;
     param_estimated = param_prev + dt*KL.inverse()*regressor.transpose()*s_error_;
}

void ModelControlCompute::UpdateWrench()
{
    //Let's Calculate the regressor matrix


    Eigen::Matrix6d lin_dampling_regressor =  velocity_measure_.asDiagonal();

    Eigen::Matrix6d quad_dampling_regressor = velocity_measure_.array().square().matrix().asDiagonal();

    Eigen::Vector6d v = velocity_measure_;
    Eigen::Vector6d acc = (velocity_measure_ - vel_prev)/dt;
    vel_prev = velocity_measure_;
    Eigen::Matrix6d  added_effect_regressor;
    added_effect_regressor << acc(0), v(1)*v(5), -v(2)*v(4), 0, 0, 0,
            -v(0)*v(5), acc(1), v(2)*v(3), 0, 0, 0,
            v(0)*v(4), -v(1)*v(3), acc(2), 0, 0, 0,
            0, v(1)*v(2), -v(1)*v(2), acc(3), v(4)*v(5), -v(4)*v(5),
            -v(0)*v(2), 0, v(0)*v(2), -v(3)*v(5), acc(4), v(3)*v(5),
            v(0)*v(1), -v(0)*v(1), 0, v(3)*v(4), -v(3)*v(4), acc(5);

    Eigen::Matrix<double, 6,4> grav_regressor;
    Eigen::Vector3d e3(0.0 ,0.0 ,1.0);
    grav_regressor << pose_ang_measure_inv_.toRotationMatrix()*e3, Eigen::MatrixXd::Zero(3,3),
            Eigen::MatrixXd::Zero(3,1), skew(pose_ang_measure_inv_.toRotationMatrix()*e3) ;

    regressor << lin_dampling_regressor, quad_dampling_regressor, added_effect_regressor, grav_regressor;

    //Let's update the wrench
    Eigen::Vector6d wrench;
    Eigen::DiagonalMatrix<double, 6> K;
    K.diagonal() << kp, kp, kp, ko, ko, ko;
    wrench = KD*s_error_+ K * pose_error_ + regressor*param_estimated;

    tf::wrenchEigenToMsg(wrench,wrench_command_);


}

}


