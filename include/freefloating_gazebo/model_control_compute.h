#ifndef MODEL_CONTROL_COMPUTE_H
#define MODEL_CONTROL_COMPUTE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/WrenchStamped.h>

namespace Eigen
{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 22,22> Matrix22d;
typedef Eigen::Matrix<double, 6, 22> MatrixModel22;
typedef Eigen::Matrix<double, 22, 1> VectorModel22;
}

namespace ffg
{

//Eigen::Matrix3d skew(const Eigen::Vector3d &v);

class ModelControlCompute
{
public:
    ModelControlCompute()
    {
        KD.setZero();
        KL.setZero();
        pose_error_.setZero();
        velocity_error_.setZero();
        s_error_.setZero();
        pose_lin_setpoint_.setZero();
        pose_lin_measure_.setZero();
        param_estimated.setZero();
        regressor.setZero();
    }

    // get wrench command
    inline geometry_msgs::Wrench WrenchCommand() {return wrench_command_;}

    void ModelCompute(){
        UpdateError();
        UpdateWrench();
        UpdateParam();
    };

    void UpdateError();
    void UpdateWrench();
    void UpdateParam();

    // errors are stored in Vector3
    Eigen::Vector6d pose_error_, velocity_error_, s_error_;
    // velocities are stored in Vector 6
    Eigen::Vector6d velocity_setpoint_, velocity_measure_;
    // poses are stored in Vector3 and Quaternion
    Eigen::Vector3d pose_lin_setpoint_, pose_lin_measure_;
    Eigen::Quaterniond pose_ang_setpoint_, pose_ang_measure_inv_;

    // parameters are stored in VectorXd
    Eigen::VectorModel22 param_estimated;

    // gains of the model
    double lp, lo, kp, ko;
    Eigen::Matrix6d KD;
    Eigen::Matrix22d KL;

    // the regressor matrix
    Eigen::MatrixModel22 regressor;
private:
    // wrench command
    geometry_msgs::Wrench wrench_command_;

};

}

#endif // MODEL_CONTROL_COMPUTE_H
