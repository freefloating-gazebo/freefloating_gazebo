
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>

#include <freefloating_gazebo/freefloating_gazebo_control.h>

using std::cout;
using std::endl;
using std::string;

namespace gazebo
{

void FreeFloatingControlPlugin::ReadVector3(const std::string &_string, math::Vector3 &_vector)
{
    std::stringstream ss(_string);
    double xyz[3];
    for(unsigned int i=0;i<3;++i)
        ss >> xyz[i];
    _vector.Set(xyz[0], xyz[1], xyz[2]);
}

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
    ROS_INFO("Loading freefloating_control plugin for %s", _model->GetName().c_str());

    // get model and name
    model_ = _model;
    robot_namespace_ = model_->GetName();
    controller_is_running_ = true;

    // parse plugin topic options
    std::string joint_command_topic = "joint_command";
    std::string body_command_topic = "body_command";

    if(_sdf->HasElement("bodyCommandTopic"))  body_command_topic = _sdf->Get<std::string>("bodyCommandTopic");
    if(_sdf->HasElement("jointCommandTopic"))  joint_command_topic = _sdf->Get<std::string>("jointCommandTopic");
    if(_sdf->HasElement("link"))
        body_ = model_->GetLink(_sdf->Get<std::string>("link"));
    else
        body_ = model_->GetLinks()[0];

    // parse thruster elements
    std::vector<double> map_elem;
    map_elem.clear();
    double map_coef;
    sdf::ElementPtr sdf_element = _sdf->GetFirstElement();
    unsigned int i,j;
    while(sdf_element)
    {
        if(sdf_element->GetName() == "thruster")
        {
            //register thruster only if map is present
            if(sdf_element->HasElement("map"))
            {
                // register map coefs
                std::stringstream ss(sdf_element->Get<std::string>("map"));
                for(i=0;i<6;++i)
                {
                    ss >> map_coef;
                    map_elem.push_back(map_coef);
                }
                // register max effort
                if(sdf_element->HasElement("effort"))
                    thruster_max_command_.push_back(sdf_element->Get<double>("effort"));
                else
                    thruster_max_command_.push_back(100);
            }
        }
        sdf_element = sdf_element->GetNextElement();
    }
    // build thruster map from map elements
    thruster_nb_ = map_elem.size()/6;
    if(thruster_nb_ != 0)
    {
        thruster_map_.resize(6, thruster_nb_);
        for(i=0;i<6;++i)
            for(j=0;j<thruster_nb_;++j)
                thruster_map_(i,j) = map_elem[6*j + i];
        ComputePseudoInverse(thruster_map_, thruster_inverse_map_);
    }
    body_command_.resize(6);

    // register ROS node & time
    rosnode_ = ros::NodeHandle(robot_namespace_);

    // initialize subscriber to joint commands
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                joint_command_topic, 1,
                boost::bind(&FreeFloatingControlPlugin::JointCommandCallBack, this, _1),
                ros::VoidPtr(), &callback_queue_);
    joint_command_subscriber_ = rosnode_.subscribe(ops);
    joint_command_received_ = false;

    // initialize subscriber to body commands
    ops = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
                body_command_topic, 1,
                boost::bind(&FreeFloatingControlPlugin::BodyCommandCallBack, this, _1),
                ros::VoidPtr(), &callback_queue_);
    body_command_subscriber_ = rosnode_.subscribe(ops);
    body_command_received_ = false;

    // init service
    //switch_service_ = rosnode_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("switch", &FreeFloatingControlPlugin::SwitchService, this);


    // push control data to parameter server
    ros::NodeHandle control_node(rosnode_, "controllers");
    control_node.setParam("config/body/link", body_->GetName());
    char param[FILENAME_MAX];

    std::string axe[] = {"x", "y", "z", "roll", "pitch", "yaw"};
    std::vector<std::string> controlled_axes;
    if(thruster_nb_)
    {
        unsigned int thr_i, dir_i;

        // compute max force in each direction, writes controlled axes
        double thruster_max_effort;

        for(dir_i=0;dir_i<6;++dir_i)
        {
            thruster_max_effort = 0;
            for(thr_i=0;thr_i<thruster_nb_;++thr_i)
                thruster_max_effort += thruster_max_command_[thr_i] * std::abs(thruster_map_(dir_i,thr_i));
            if(thruster_max_effort != 0)
            {
                controlled_axes.push_back(axe[dir_i]);
                // push to position PID
                sprintf(param, "%s/%s/position/i_clamp", body_->GetName().c_str(), axe[dir_i].c_str());
                control_node.setParam(param, thruster_max_effort);
                // push to velocity PID
                sprintf(param, "%s/%s/velocity/i_clamp", body_->GetName().c_str(), axe[dir_i].c_str());
                control_node.setParam(param, thruster_max_effort);
            }
        }

        // initialize thruster use publisher
        thruster_use_.data.resize(thruster_max_command_.size());
        thruster_use_publisher_ = rosnode_.advertise<std_msgs::Float32MultiArray>("thruster_use", 1);


    }
    else
    {
        // no thrusters? assume all axes are controlled
        for(unsigned int i=0;i<6;++i)
            controlled_axes.push_back(axe[i]);
    }
    // push controlled axes
    control_node.setParam("config/body/axes", controlled_axes);


    // push joint limits and setup joint states
    if(model_->GetJointCount() != 0)
    {
        std::vector<std::string> joint_names;
        std::vector<double> joint_min, joint_max;
        std::string name;
        physics::JointPtr joint;
        bool cascaded_position = true;
        if(control_node.hasParam("config/joints/cascaded_position"))
            control_node.getParam("config/joints/cascaded_position", cascaded_position);

        for(i=0;i<model_->GetJointCount();++i)
        {

            joint = model_->GetJoints()[i];
            name = joint->GetName();

            // set max velocity or max effort for the position PID
            sprintf(param, "%s/position/i_clamp", name.c_str());
            if(cascaded_position)
                control_node.setParam(param, joint->GetVelocityLimit(0));
            else
                control_node.setParam(param, joint->GetEffortLimit(0));

            // set max effort for the velocity PID
            sprintf(param, "%s/velocity/i_clamp", name.c_str());
            control_node.setParam(param, joint->GetEffortLimit(0));

            // save name and joint limits
            joint_names.push_back(name);
            joint_min.push_back(joint->GetLowerLimit(0).Radian());
            joint_max.push_back(joint->GetUpperLimit(0).Radian());
        }

        // push setpoint topic, name, lower and bound
        control_node.setParam("config/joints/name", joint_names);
        control_node.setParam("config/joints/lower", joint_min);
        control_node.setParam("config/joints/upper", joint_max);

        // setup joint_states publisher
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("joint_states", 1);
        joint_states_.name = joint_names;
        joint_states_.position.resize(model_->GetJointCount());
        joint_states_.velocity.resize(model_->GetJointCount());
    }


    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&FreeFloatingControlPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Loaded freefloating_control plugin for %s.", _model->GetName().c_str());
}

void FreeFloatingControlPlugin::Update()
{
    // activate callbacks
    callback_queue_.callAvailable();

    if(controller_is_running_)
    {

        // deal with joint control
        if(joint_command_received_)
        {
            physics::JointPtr joint;
            for(unsigned int i=0;i<model_->GetJointCount();++i)
            {
                joint = model_->GetJoints()[i];
                joint->SetForce(0,joint_command_.effort[i] - joint->GetDamping(0) * joint->GetVelocity(0));
            }
        }

        // deal with body control
        if(body_command_received_)
        {
            //body_->AddRelativeForce(math::Vector3(body_command_(0), body_command_(1), body_command_(2)));
            // apply this wrench
            body_->AddForceAtWorldPosition(body_->GetWorldPose().rot.RotateVector(math::Vector3(body_command_(0), body_command_(1), body_command_(2))), body_->GetWorldCoGPose().pos);

            //body_pose.rot.RotateVector(math::Vector3(body_command_(0), body_command_(1), body_command_(2))), body_pose.pos);
            //     body_->AddForceAtRelativePosition(, math::Vector3(0,0,0));
            body_->AddRelativeTorque(math::Vector3(body_command_(3), body_command_(4), body_command_(5)));

            // publish thruster use
            thruster_use_publisher_.publish(thruster_use_);
        }
    }

    // publish joint states anyway
    if(model_->GetJointCount() != 0)
    {
        for(unsigned int i=0;i<model_->GetJointCount();++i)
        {
            joint_states_.position[i] = model_->GetJoints()[i]->GetAngle(0).Radian();   
            joint_states_.velocity[i] = model_->GetJoints()[i]->GetVelocity(0);
            physics::JointPtr j;
        }
        joint_state_publisher_.publish(joint_states_);
    }


}


void FreeFloatingControlPlugin::ComputePseudoInverse(const Eigen::MatrixXd &_M, Eigen::MatrixXd &_pinv_M)
{
    _pinv_M.resize(_M.cols(), _M.rows());
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_M(_M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd dummy_in;
    Eigen::VectorXd dummy_out(_M.cols());
    unsigned int i,j;
    for(i=0;i<_M.rows();++i)
    {
        dummy_in = Eigen::VectorXd::Zero(_M.rows());
        dummy_in(i) = 1;
        dummy_out = svd_M.solve(dummy_in);
        for(j = 0; j<_M.cols();++j)
            _pinv_M(j,i) = dummy_out(j);
    }
}

void FreeFloatingControlPlugin::BodyCommandCallBack(const geometry_msgs::WrenchConstPtr &_msg)
{
    body_command_received_ = true;

    // store body command
    body_command_(0) = _msg->force.x;
    body_command_(1) = _msg->force.y;
    body_command_(2) = _msg->force.z;
    body_command_(3) = _msg->torque.x;
    body_command_(4) = _msg->torque.y;
    body_command_(5) = _msg->torque.z;

    // if thrusters, modify body command depending on thruster saturation
    if(thruster_nb_)
    {
        //compute corresponding thruster command
        Eigen::VectorXd thruster_command = thruster_inverse_map_ * body_command_;

        // get maximum thruster use
        double norm_ratio = 1;
        unsigned int i;
        for(i=0;i<thruster_nb_;++i)
            norm_ratio = std::max(norm_ratio, std::abs(thruster_command(i)) / thruster_max_command_[i]);

        // go back in body command
        body_command_ = 1/norm_ratio * thruster_map_ * thruster_command;

        // also store thruster use in %
        for(i=0;i<thruster_nb_;++i)
            thruster_use_.data[i] = 100/norm_ratio*std::abs(thruster_command(i) / thruster_max_command_[i]);

    }

}
}
