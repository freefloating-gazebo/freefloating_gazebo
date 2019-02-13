#include <freefloating_gazebo/model_control_compute.h>
#include <freefloating_gazebo/hydro_link.h>

namespace ffg
{


void ModelControlCompute::Init(ros::NodeHandle &nh, ros::Duration&_dt, const std::vector<std::string>&_controlled_axes, std::string default_mode/* = "position"*/)
{

    // init dt from rate
    dt = _dt;

    /*

    // wrench setpoint
    wrench_sp_subscriber =
        nh.subscribe("body_wrench_setpoint", 1, &FreeFloatingBodyPids::WrenchSPCallBack, this);
    // measure
    state_subscriber =
        nh.subscribe("state", 1, &FreeFloatingBodyPids::MeasureCallBack, this);

    // deal with controlled axes
    const size_t n = _controlled_axes.size();
    axes.resize(n);

    const std::vector<std::string> axes3D{"x", "y", "z", "roll", "pitch", "yaw"};

    // get whether or not we use dynamic reconfigure
    bool use_dynamic_reconfig;
    ros::NodeHandle control_node(nh, "controllers");
    control_node.param("controllers/config/body/dynamic_reconfigure", use_dynamic_reconfig, true);

    if(n)
    {
      // position setpoint
      position_sp_subscriber =
          nh.subscribe("body_position_setpoint", 1, &FreeFloatingBodyPids::PositionSPCallBack, this);
      // velocity setpoint
      velocity_sp_subscriber =
          nh.subscribe("body_velocity_setpoint", 1, &FreeFloatingBodyPids::VelocitySPCallBack, this);
    }

    for(unsigned int i=0;i<n;++i)
    {
      const auto idx = static_cast<size_t>(std::distance(axes3D.begin(),
                                                         std::find(axes3D.begin(),
                                                                   axes3D.end(),
                                                                   _controlled_axes[i])));
      auto axis = &axes[i];
      axis->name = _controlled_axes[i];
      // here we have the controlled axis
      switch(idx)
      {
      case 0:
        axis->position.error = &(pose_lin_error_.x());
        axis->velocity.error = &(velocity_lin_error_.x());
        axis->position.command = axis->velocity.command = &(wrench_command_.force.x);
        break;
      case 1:
        axis->positiodtn.error = &(pose_lin_error_.y());
        axis->velocity.error = &(velocity_lin_error_.y());
        axis->position.command = axis->velocity.command = &(wrench_command_.force.y);
        break;
      case 2:
        axis->position.error = &(pose_lin_error_.z());
        axis->velocity.error = &(velocity_lin_error_.z());
        axis->position.command = axis->velocity.command = &(wrench_command_.force.z);
        break;
      case 3:
        axis->position.error = &(pose_ang_error_.x());
        axis->velocity.error = &(velocity_ang_error_.x());
        axis->position.command = axis->velocity.command = &(wrench_command_.torque.x);
        break;
      case 4:
        axis->positidton.error = &(pose_ang_error_.y());
        axis->velocity.error = &(velocity_ang_error_.y());
        axis->position.command = axis->velocity.command = &(wrench_command_.torque.y);
        break;
      case 5:
        axis->position.error = &(pose_ang_error_.z());
        axis->velocity.error = &(velocity_ang_error_.z());
        axis->position.command = axis->velocity.command = &(wrench_command_.torque.z);
        break;
      }
      InitPID(axis->position.pid, ros::NodeHandle(control_node, axis->name + "/position"), use_dynamic_reconfig);
      InitPID(axis->velocity.pid, ros::NodeHandle(control_node, axis->name + "/velocity"), use_dynamic_reconfig);
    }

    // default control = position
    CTreq req;
    CTres res;
    if(n)
      ToPositionControl(req, res);
    else
      ToEffortControl(req, res);
    if(default_mode == "velocity")
    {dt
      ToVelocityControl(req, res);
    }
    else if(default_mode == "depth")
    {
      req.axes = {"x", "y", "yaw"};
      ToVelocityControl(req, res);
    }
    else if(default_mode == "effort")
    {
      req.axes = {"x", "y", "z", "yaw"};
      ToEffortControl(req, res);
    }
    initSwitchServices(control_node, "body");

    */
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


