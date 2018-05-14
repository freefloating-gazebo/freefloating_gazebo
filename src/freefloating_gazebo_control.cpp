
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <freefloating_gazebo/freefloating_gazebo_control.h>
#include <chrono>
#include <thread>
#include <functional>

using std::cout;
using std::endl;
using std::string;
using ignition::math::Vector3d;

#if GAZEBO_MAJOR_VERSION < 9
#define GAZEBOLD
    ignition::math::Vector3d v3convert(gazebo::math::Vector3 src)
    {
        return ignition::math::Vector3d(src.x, src.y, src.z);
    }
#endif

namespace gazebo
{

bool FreeFloatingControlPlugin::SwitchService(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    if(controller_is_running_)
        ROS_INFO("Switching freefloating_control OFF");
    else
        ROS_INFO("Switching freefloating_control ON");
    controller_is_running_ = !controller_is_running_;
    return true;
}

void FreeFloatingControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // get model and name
    model_ = _model;
    robot_namespace_ = model_->GetName();
    controller_is_running_ = true;

    // register ROS node & time
    rosnode_ = ros::NodeHandle(robot_namespace_);
    ros::NodeHandle control_node(rosnode_, "controllers");
    t_prev_ = 0;

    // get surface Z
    rosnode_.getParam("/gazebo/surface", z_surface_);

    // look for thrusters
    if(_sdf->HasElement("link"))
        body_ = model_->GetLink(_sdf->Get<std::string>("link"));
    else
        body_ = model_->GetLinks()[0];

    // parse thruster elements
    mapper_.parse(_sdf->ToString(""));
    // add steering thrusters if any
    thruster_links_.clear();
    for(auto name: mapper_.names)
    {
        for(auto link: _model->GetLinks())
        {
            if(link->GetName() == name)
            {
                thruster_links_.push_back(link);
                break;
            }
        }
    }


    control_body_ = true;
    if(!mapper_.names.size())
    {
        ROS_INFO("No thrusters found in %s", _model->GetName().c_str());
        control_body_ = false;
    }

    // *** SET UP BODY CONTROL
    char param[FILENAME_MAX];
    std::string body_command_topic, body_state_topic;
    unsigned int i,j;
    if(control_body_)
    {
        // read topics
        control_node.param("config/body/command", body_command_topic, std::string("body_command"));
        control_node.param("config/body/state", body_state_topic, std::string("body_state"));

        wrench_control_ = false;
        // get control type (wrench / thruster) if explicit
        if(control_node.hasParam("config/body/type"))
        {
            std::string control_type;
            control_node.getParam("config/body/type", control_type);
            wrench_control_ = (control_type == "wrench");
        }
        else   // guess control type from PID gains (gains assume wrench control)
        {
            for(auto axe: {"x", "y", "z", "roll", "pitch","yaw"})
            {
                if(control_node.hasParam(axe))
                    wrench_control_ = true;
            }
        }

        // check consistency
        if(wrench_control_ && mapper_.steer_idx.size())
        {
            ROS_WARN("%s has steering thrusters and cannot be controlled with wrench", model_->GetName().c_str());
            wrench_control_ = false;
        }

        // initialize subscriber to body commands
        thruster_command_.resize(mapper_.names.size());
        // initialize publisher to thruster_use
        thruster_use_.name = mapper_.names;
        thruster_use_.position.resize(mapper_.names.size());
        thruster_use_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("thruster_use", 1);

        ros::SubscribeOptions ops;
        if(wrench_control_)
        {
            ops = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
                        body_command_topic, 1,
                        boost::bind(&FreeFloatingControlPlugin::BodyCommandCallBack, this, _1),
                        ros::VoidPtr(), &callback_queue_);
        }
        else
        {
            ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                        body_command_topic, 1,
                        boost::bind(&FreeFloatingControlPlugin::ThrusterCommandCallBack, this, _1),
                        ros::VoidPtr(), &callback_queue_);
        }
        body_command_subscriber_ = rosnode_.subscribe(ops);
        body_command_received_ = false;

        // if wrench control, compute map pseudo-inverse and help PID's with maximum forces by axes
        std::vector<std::string> axe = {"x", "y", "z", "roll", "pitch", "yaw"};
        std::vector<std::string> controlled_axes;
        if(wrench_control_ && mapper_.fixed_idx.size() && !mapper_.steer_idx.size())
        {
            mapper_.initWrenchControl(control_node, body_->GetName(), axe, controlled_axes);
        }
        else
        {
            // thruster-controlled? assume all axes can be controlled
            for(unsigned int i=0;i<6;++i)
                controlled_axes.push_back(axe[i]);
        }
        // push controlled axes
        control_node.setParam("config/body/axes", controlled_axes);
    }

    // *** END BODY CONTROL

    // *** JOINT CONTROL
    joints_.clear();
    // check for joint param
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    control_joints_ = control_node.hasParam("config/joints");

    if(control_joints_ && model_->GetJointCount() != 0)
    {
        std::string joint_command_topic, joint_state_topic;
        control_node.param("config/joints/command", joint_command_topic, std::string("joint_command"));
        control_node.param("config/joints/state", joint_state_topic, std::string("joint_state"));

        // initialize subscriber to joint commands
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    joint_command_topic, 1,
                    boost::bind(&FreeFloatingControlPlugin::JointCommandCallBack, this, _1),
                    ros::VoidPtr(), &callback_queue_);
        joint_command_subscriber_ = rosnode_.subscribe(ops);
        joint_command_received_ = false;

        // push joint limits and setup joint states
        std::vector<std::string> joint_names;
        std::vector<double> joint_min, joint_max, vel_max;
        std::string name;
        physics::JointPtr joint;
        bool cascaded_position = true;
        if(control_node.hasParam("config/joints/cascaded_position"))
            control_node.getParam("config/joints/cascaded_position", cascaded_position);

        for(i=0;i<model_->GetJointCount();++i)
        {
            joint = model_->GetJoints()[i];
            name = joint->GetName();

            if(control_node.hasParam(name))
            {
                joints_.push_back(joint);
                // set max velocity or max effort for the position PID
                sprintf(param, "%s/position/i_clamp", name.c_str());
                if(cascaded_position)
                    control_node.setParam(param, joint->GetVelocityLimit(0));
                else
                    control_node.setParam(param, joint->GetEffortLimit(0));

                // set max effort for the velocity PID
                sprintf(param, "%s/velocity/i_clamp", name.c_str());
                control_node.setParam(param, joint->GetEffortLimit(0));

                // set antiwindup to true - why would anyone set it to false?
                sprintf(param, "%s/position/antiwindup", name.c_str());
                control_node.setParam(param, true);
                sprintf(param, "%s/velocity/antiwindup", name.c_str());
                control_node.setParam(param, true);

                // save name and joint limits
                joint_names.push_back(name);
#ifdef GAZEBOLD
                joint_min.push_back(joint->GetLowerLimit(0).Radian());
                joint_max.push_back(joint->GetUpperLimit(0).Radian());
#else
                joint_min.push_back(joint->LowerLimit());
                joint_max.push_back(joint->UpperLimit());
#endif
                vel_max.push_back(joint->GetVelocityLimit(0));
            }
        }

        // push setpoint topic, name, lower and bound
        control_node.setParam("config/joints/name", joint_names);
        control_node.setParam("config/joints/lower", joint_min);
        control_node.setParam("config/joints/upper", joint_max);
        control_node.setParam("config/joints/velocity", vel_max);

        // setup joint_states publisher
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
        joint_states_.name = joint_names;
        joint_states_.position.resize(joints_.size());
        joint_states_.velocity.resize(joints_.size());
    }
    // *** END JOINT CONTROL


    // set up switch service between position and velocity control
    //switch_service_ = rosnode_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("switch", &FreeFloatingControlPlugin::SwitchService, this);

    // store update rate
    if(_sdf->HasElement("updateRate"))
        update_T_ = 1./_sdf->Get<double>("updateRate");
    else
        update_T_ = 0;

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(std::bind(&FreeFloatingControlPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started FreeFloating Control Plugin for %s.", _model->GetName().c_str());
}

void FreeFloatingControlPlugin::Update()
{
    // activate callbacks
    callback_queue_.callAvailable();

    if(controller_is_running_)
    {
        // deal with joint control
        if(control_joints_ && joint_command_received_)
        {
            physics::JointPtr joint;
            unsigned int idx;
            for(unsigned int i=0;i<joint_command_.name.size();++i)
            {
                // find corresponding model joint
                idx = std::distance(joint_states_.name.begin(), std::find(joint_states_.name.begin(), joint_states_.name.end(), joint_command_.name[i]));
                joint = joints_[idx];
                joint->SetForce(0,joint_command_.effort[i]);
            }
        }

        // deal with body control if underwater
#ifdef GAZEBOLD
        if(control_body_ && body_command_received_ && (body_->GetWorldCoGPose().pos.z < z_surface_))
#else
        if(control_body_ && body_command_received_ && (body_->WorldCoGPose().Pos().Z() < z_surface_))
#endif
        {
            mapper_.saturate(thruster_command_);

            // build and apply wrench for fixed thrusters
            if(mapper_.fixed_idx.size())
            {
                Eigen::VectorXd fixed(mapper_.fixed_idx.size());
                for(unsigned int i=0;i<mapper_.fixed_idx.size();++i)
                    fixed(i) = thruster_command_(mapper_.fixed_idx[i]);
                // to wrench
                fixed = mapper_.map * fixed;
                // apply this wrench to body
#ifdef GAZEBOLD
               body_->AddForceAtWorldPosition(body_->GetWorldPose().rot.RotateVector(Vector3d(fixed(0), fixed(1), fixed(2))), body_->GetWorldCoGPose().pos);
#else
               body_->AddForceAtWorldPosition(body_->WorldPose().Rot().RotateVector(Vector3d(fixed(0), fixed(1), fixed(2))), body_->WorldCoGPose().Pos());
#endif
                body_->AddRelativeTorque(Vector3d(fixed(3), fixed(4), fixed(5)));
            }

            // apply command for steering thrusters
            if(mapper_.steer_idx.size())
            {
                for(unsigned int i=0;i<mapper_.steer_idx.size();++i)
                    thruster_links_[i]->AddRelativeForce(Vector3d(0,0,-thruster_command_(mapper_.steer_idx[i])));
            }

            // compute and publish thruster use in %
            for(int i=0;i<thruster_command_.size();++i)
                thruster_use_.position[i] = 100*std::abs(thruster_command_(i) / mapper_.max_command[i]);
            thruster_use_publisher_.publish(thruster_use_);
        }
    }

    // publish joint states anyway
    double t = ros::Time::now().toSec();
    if((t-t_prev_) > update_T_ && joints_.size() != 0)
    {
        t_prev_ = t;
        joint_states_.header.stamp = ros::Time::now();

        for(unsigned int i=0;i<joints_.size();++i)
        {
#ifdef GAZEBOLD
            joint_states_.position[i] = joints_[i]->GetAngle(0).Radian();
#else
            joint_states_.position[i] = joints_[i]->Position();
#endif
            joint_states_.velocity[i] = joints_[i]->GetVelocity(0);
        }
        joint_state_publisher_.publish(joint_states_);
    }
}


void FreeFloatingControlPlugin::BodyCommandCallBack(const geometry_msgs::WrenchConstPtr &_msg)
{
    if(!control_body_)
        return;
    body_command_received_ = true;

    // compute corresponding thruster command
    Eigen::VectorXd body_command(6);
    body_command << _msg->force.x, _msg->force.y, _msg->force.z, _msg->torque.x, _msg->torque.y, _msg->torque.z;

    thruster_command_ = mapper_.inverse_map * body_command;
}


void FreeFloatingControlPlugin::ThrusterCommandCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
    if(!control_body_)
        return;

    const bool read_effort = _msg->effort.size();

    if(read_effort && (_msg->name.size() != _msg->effort.size()))
    {
        ROS_WARN("Received inconsistent thruster command, name and effort dimension do not match");
        return;
    }
    else
        if(!read_effort && (_msg->name.size() != _msg->position.size()))
        {
            ROS_WARN("Received inconsistent thruster command, name and position dimension do not match");
            return;
        }

    body_command_received_ = true;
    // store thruster command
    for(unsigned int i=0;i<mapper_.names.size();++i)
    {
        for(unsigned int j=0;j<_msg->name.size();++j)
        {
            if(mapper_.names[i] == _msg->name[j])
                thruster_command_(i) = read_effort?_msg->effort[j]:_msg->position[j];
        }
    }
}




}   // namespace gazebo
