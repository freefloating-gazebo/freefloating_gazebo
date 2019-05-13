#ifndef MODEL_CONTROL_COMPUTE_H
#define MODEL_CONTROL_COMPUTE_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <freefloating_gazebo/BrovConfig.h>
#include <freefloating_gazebo/thruster_allocator.h>

namespace Eigen
{
typedef Eigen::DiagonalMatrix<double, 6, 6> Matrix6dd;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::DiagonalMatrix<double, 20,20> Matrix20dd;
typedef Eigen::Matrix<double, 6, 20> MatrixModel20;
typedef Eigen::Matrix<double, 20, 1> VectorModel20;
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

    void Init(ros::NodeHandle &nh,
              ros::Duration&_dt,
              const HydroLink base_link,
              std::string default_mode = "position");

    bool Update(free_floating_gazebo::BrovConfig &config, bool reconfig_update){
        if(reconfig_update)
        {
            Dyn_update(config);
            reconfig_update = false;
        }

        if(UpdateError())
        {
            UpdateParam();
            UpdateWrench();
            return true;
        }else
            return false;
    };

    bool UpdateError();
    void UpdateParam();
    void UpdateWrench();

    void GetGains(const ros::NodeHandle &control_node);

    void InitParam(HydroLink base_link);

    void SetGainsK()
    {
        K.diagonal() << kp, kp, kp, ko, ko, ko;
    }

    void SetGainsKD(double d1, double d2)
    {
        KD.diagonal() <<    d1, d1, d1,
                d2, d2, d2;
    }

    void SetGainsKL(std::vector<double> &diag)
    {

        KL.diagonal() <<    diag.at(0), diag.at(0), diag.at(0), diag.at(0), diag.at(0), diag.at(0),
                diag.at(1), diag.at(1), diag.at(1), diag.at(1), diag.at(1), diag.at(1),
                diag.at(2), diag.at(2), diag.at(2), diag.at(2), diag.at(2), diag.at(2),
                diag.at(3), diag.at(3);
    }

    // get wrench command
    inline geometry_msgs::Wrench WrenchCommand() {return wrench_command_;}

    // parse received position setpoint
    void PositionSPCallBack(const geometry_msgs::PoseStampedConstPtr& _msg);
    // parse received velocity setpoint
    void VelocitySPCallBack(const geometry_msgs::TwistStampedConstPtr & _msg);
    // parse received wrench
    void WrenchSPCallBack(const geometry_msgs::WrenchStampedConstPtr & _msg)
    {
        wrench_command_ = _msg->wrench;
    }
    // parse received body measure
    void MeasureCallBack(const nav_msgs::OdometryConstPtr& _msg);

    void Dyn_update(free_floating_gazebo::BrovConfig &config)
    {
        lo = config.lo;
        lp = config.lp;
        kp = config.kp;
        ko = config.ko;
        SetGainsK();
        SetGainsKD(config.kd1,config.kd2);
        std::vector<double> diag;
        diag.push_back(config.kl1);
        diag.push_back(config.kl2);
        diag.push_back(config.kl3);
        diag.push_back(config.kl4);
        SetGainsKL(diag);
    }

    // errvelocity_error_ors are stored in Vector3
    Eigen::Vector6d pose_error_, velocity_error_, s_error_;
    // velocities are stored in Vector 6
    Eigen::Vector6d velocity_setpoint_, velocity_measure_, vel_prev;
    // poses are stored in Vector3 and Quaternion
    Eigen::Vector3d pose_lin_setpoint_, pose_lin_measure_;
    Eigen::Quaterniond pose_ang_setpoint_, pose_ang_measure_inv_;

    // parameters are stored in VectorXd
    Eigen::VectorModel20 param_estimated, param_prev;

    // gains of the model
    double lp, lo, kp, ko;
    Eigen::Matrix6dd KD;
    Eigen::Matrix20dd KL;
    Eigen::DiagonalMatrix<double, 6> K;

    // the regressor matrix
    Eigen::MatrixModel20 regressor;

    //
    bool state_received = false, setpoint_position_ok = false, setpoint_velocity_ok = false;
    // time between two updates
    ros::Duration dt;

private:

    ros::Subscriber position_sp_subscriber, velocity_sp_subscriber,
    wrench_sp_subscriber, state_subscriber;

    // wrench command
    geometry_msgs::Wrench wrench_command_;

};

}

#endif // MODEL_CONTROL_COMPUTE_H
