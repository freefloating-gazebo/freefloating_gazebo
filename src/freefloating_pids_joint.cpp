#include <freefloating_gazebo/freefloating_pids_joint.h>

using std::cout;
using std::endl;
using std::string;


void FreeFloatingJointPids::Init(const ros::NodeHandle&_node,
                                 ros::Duration&_dt,
                                 std::string default_mode)
{
    // init dt from rate
    dt_ = _dt;
    pid_node_ = _node;

    // store dynamic reconfigure

    // filter param
    alpha_ = .5;

    // get params from server
    _node.getParam("config/joints/name", axes_);

    // get whether or not we use a cascaded controller for position
    _node.param("config/joints/cascaded_position", cascaded_position_, true);

    // get whether or not we use dynamic reconfigure
    bool use_dynamic_reconfig;
    _node.param("config/joints/dynamic_reconfigure", use_dynamic_reconfig, false);

    // resize vectors
    const unsigned int n = axes_.size();
    position_pids_.resize(n);
    velocity_pids_.resize(n);
    position_error_.resize(n);
    velocity_error_.resize(n);
    position_filtered_measure_.resize(n);
    velocity_filtered_measure_.resize(n);

    joint_measure_.velocity.resize(n);
    joint_measure_.position.resize(n);
    joint_setpoint_.velocity.resize(n);
    joint_setpoint_.position.resize(n);
    joint_setpoint_.name = axes_;
    joint_command_.name = axes_;
    joint_command_.effort.resize(n);

    // store limits
    _node.getParam("config/joints/upper", joint_upper_);
    _node.getParam("config/joints/lower", joint_lower_);
    _node.getParam("config/joints/velocity", joint_max_velocity_);
    vmax_is_set_ = !cascaded_position_;

    for(unsigned int i=0;i<n;++i)
    {
        position_pids_[i].error_ptr = &(position_error_[i]);
        velocity_pids_[i].error_ptr = &(velocity_error_[i]);
        velocity_pids_[i].command_ptr = &(joint_command_.effort[i]);

        // position PID output depends on control type
        if(cascaded_position_)
            position_pids_[i].command_ptr = &(joint_setpoint_.velocity[i]);
        else
            position_pids_[i].command_ptr = &(joint_command_.effort[i]);

        position_filtered_measure_[i] = velocity_filtered_measure_[i] = 0;

        InitPID(position_pids_[i].pid, ros::NodeHandle(_node, axes_[i] + "/position"), use_dynamic_reconfig);
        InitPID(velocity_pids_[i].pid, ros::NodeHandle(_node, axes_[i] + "/velocity"), use_dynamic_reconfig);

    }
    initSwitchServices("joints");

    // init to default control
    CTreq req;
    CTres res;

    if(default_mode == "velocity")
        ToVelocityControl(req, res);
    else if(default_mode == "effort")
        ToEffortControl(req, res);
    else
        ToPositionControl(req, res);
}



bool FreeFloatingJointPids::UpdatePID()
{
    if(!vmax_is_set_)
    {
        control_toolbox::Pid::Gains gains;

        for(unsigned int i=0;i<velocity_error_.size();++i)
        {
               gains = position_pids_[i].pid.getGains();
               joint_max_velocity_[i] = gains.i_max_;
        }
        vmax_is_set_ = true;
    }

    if(state_received_)
    {
        // do position PID computations only if needed
        if(setpoint_position_ok_ && position_used_)
        {
            //  cout << "Joint position error: ";
            // get position error
            for(unsigned int i=0;i<position_error_.size();++i)
            {
                position_filtered_measure_[i] = filters::exponentialSmoothing(joint_measure_.position[i], position_filtered_measure_[i], alpha_);
                position_error_[i] = filters::clamp(joint_setpoint_.position[i], joint_lower_[i], joint_upper_[i]) - position_filtered_measure_[i];
                //      cout << position_error_[i];
            }
            //   cout << endl;
            // update pid's
            UpdatePositionPID();
            // has written new velocity setpoint
        }

        if(setpoint_velocity_ok_ && velocity_used_)
        {
            // get velocity error
            for(unsigned int i=0;i<velocity_error_.size();++i)
            {
                // if joint is a max bound, error is 0
                if((joint_measure_.position[i] >= joint_upper_[i] && joint_setpoint_.velocity[i] > 0) ||
                        (joint_measure_.position[i] <= joint_lower_[i] && joint_setpoint_.velocity[i] < 0))
                    velocity_error_[i] = 0;
                else
                {
                    velocity_filtered_measure_[i] = filters::exponentialSmoothing(joint_measure_.velocity[i], velocity_filtered_measure_[i], alpha_);
                    //  cout << "Joint " << i+1 << "vel: " << joint_setpoint_.velocity[i] << ", min: " << -joint_max_velocity_[i] << ", max: " << joint_max_velocity_[i] << endl;
                    velocity_error_[i] = filters::clamp(joint_setpoint_.velocity[i], -joint_max_velocity_[i], joint_max_velocity_[i]) - velocity_filtered_measure_[i];
                }
            }

            // update pid's
            UpdateVelocityPID();
        }

        return true;
    }

    return false;
}

void FreeFloatingJointPids::SetpointCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
    // check for joint ordering


    for(int i=0;i<joint_setpoint_.name.size();++i)
    {
        // get corresponding index in message
        int idx = std::distance(_msg->name.begin(),
                                std::find(_msg->name.begin(),
                                          _msg->name.end(),
                                          joint_setpoint_.name[i]));
        if(idx < _msg->name.size())
        {
            if(_msg->position.size() > idx)
            {
                joint_setpoint_.position[i] = _msg->position[idx];
                setpoint_position_ok_ = true;
            }
            if(_msg->velocity.size() > idx)
            {
                joint_setpoint_.velocity[i] = _msg->velocity[idx];
                setpoint_velocity_ok_ = true;
            }
            if(_msg->effort.size() > idx)
                joint_command_.effort[i] = _msg->effort[idx];
        }
    }
}



void FreeFloatingJointPids::MeasureCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
    state_received_ = true;
    // assume joints are ordered the same, should be the case when the measure comes from Gazebo
    joint_measure_ = *_msg;
}
