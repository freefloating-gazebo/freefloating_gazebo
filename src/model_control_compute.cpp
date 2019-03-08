#include <freefloating_gazebo/model_control_compute.h>
#include <freefloating_gazebo/hydro_link.h>

namespace ffg
{


void ModelControlCompute::Init(ros::NodeHandle &nh, ros::Duration&_dt, const std::vector<std::string>&_controlled_axes, std::string default_mode/* = "position"*/)
{

    // init dt from rate
    dt = _dt;

    // wrench setpoint
    wrench_sp_subscriber =
            nh.subscribe("body_wrench_setpoint", 1, &ModelControlCompute::WrenchSPCallBack, this);
    // measure
    state_subscriber =
            nh.subscribe("state", 1, &ModelControlCompute::MeasureCallBack, this);

    // deal with controlled axes
    const size_t n = _controlled_axes.size();
    //axes.resize(n);

    //const std::vector<std::string> axes3D{"x", "y", "z", "roll", "pitch", "yaw"};

    // get whether or not we use dynamic reconfigure
    //bool use_dynamic_reconfig;
    //ros::NodeHandle control_node(nh, "controllers");
    //control_node.param("controllers/config/body/dynamic_reconfigure", use_dynamic_reconfig, true);

    if(n)//If we cans actually control something
    {
        if(default_mode == "position"){
            // position setpoint
            position_sp_subscriber =
                    nh.subscribe("body_position_setpoint", 1, &ModelControlCompute::PositionSPCallBack, this);
        }
        else if(default_mode == "velocity"){
            // velocity setpoint
            velocity_sp_subscriber =
                    nh.subscribe("body_velocity_setpoint", 1, &ModelControlCompute::VelocitySPCallBack, this);
        }
        //TODO initialize setpoint (angular desired value in both cases);
    }

    //TODO : default_mode should determine which are the dof we control
    //TODO : how from the Init function we shall define to the rest of the program what we are going to take into account ?
}

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
    param_estimated = param_prev + dt.toSec()*KL.inverse()*regressor.transpose()*s_error_;
}

void ModelControlCompute::UpdateWrench()
{
    //Let's Calculate the regressor matrix


    Eigen::Matrix6d lin_dampling_regressor =  velocity_measure_.asDiagonal();

    Eigen::Matrix6d quad_dampling_regressor = velocity_measure_.array().square().matrix().asDiagonal();

    Eigen::Vector6d v = velocity_measure_;
    Eigen::Vector6d acc = (velocity_measure_ - vel_prev)/dt.toSec();
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


void ModelControlCompute::PositionSPCallBack(const geometry_msgs::PoseStampedConstPtr& _msg)
{
    setpoint_position_ok = true;

    pose_lin_setpoint_ = Eigen::Vector3d(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
    pose_ang_setpoint_ = Eigen::Quaterniond(_msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z);
}

void ModelControlCompute::VelocitySPCallBack(const geometry_msgs::TwistStampedConstPtr & _msg)
{
    setpoint_velocity_ok = true;
    Eigen::Vector3d velocity_lin_setpoint_ = Eigen::Vector3d(_msg->twist.linear.x, _msg->twist.linear.y, _msg->twist.linear.z);
    Eigen::Vector3d velocity_ang_setpoint_ = Eigen::Vector3d(_msg->twist.angular.x, _msg->twist.angular.y, _msg->twist.angular.z);
    velocity_setpoint_ << velocity_lin_setpoint_, velocity_ang_setpoint_;
}

void ModelControlCompute::MeasureCallBack(const nav_msgs::OdometryConstPtr &_msg)
{
    state_received = true;
    // positions are expressed in the world frame, rotation is inversed
    pose_lin_measure_ = Eigen::Vector3d(_msg->pose.pose.position.x, _msg->pose.pose.position.y, _msg->pose.pose.position.z);
    pose_ang_measure_inv_ = Eigen::Quaterniond(_msg->pose.pose.orientation.w, _msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z).inverse();

    // change velocities from world to body frame

    Eigen::Vector3d velocity_lin_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.linear.x, _msg->twist.twist.linear.y, _msg->twist.twist.linear.z);
    Eigen::Vector3d velocity_ang_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.angular.x, _msg->twist.twist.angular.y, _msg->twist.twist.angular.z);
    velocity_measure_ << velocity_lin_measure_, velocity_ang_measure_;// TODO be sure it can be done
}


}


