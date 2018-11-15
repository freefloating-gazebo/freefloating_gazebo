#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>

#include <freefloating_gazebo/freefloating_gazebo_fluid.h>

using std::cout;
using std::endl;
using std::string;

#if GAZEBO_MAJOR_VERSION < 9
#define GAZEBOLD
    ignition::math::Vector3d v3convert(gazebo::math::Vector3 src)
    {
        return ignition::math::Vector3d(src.x, src.y, src.z);
    }
#endif

namespace gazebo
{


void FreeFloatingFluidPlugin::ReadVector3(const std::string &_string, Vector3d &_vector)
{
    std::stringstream ss(_string);
    double xyz[3];
    for(unsigned int i=0;i<3;++i)
        ss >> xyz[i];
    _vector.Set(xyz[0], xyz[1], xyz[2]);
}

void FreeFloatingFluidPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    ROS_INFO("Loading freefloating_fluid plugin");
    this->world_ = _world;

    // register ROS node
    rosnode_ = new ros::NodeHandle("gazebo");

    // parse plugin options
    description_ = "robot_description";
    has_surface_ = false;
    surface_plane_.Set(0,0,1,0); // default ocean surface plane is Z=0
    std::string fluid_topic = "current";

    if(_sdf->HasElement("descriptionParam"))  description_ = _sdf->Get<std::string>("descriptionParam");
    if(_sdf->HasElement("surface"))
    {
        has_surface_ = true;
        // get one surface point
        Vector3d surface_point;
        ReadVector3(_sdf->Get<std::string>("surface"), surface_point);
        // get gravity
#ifdef GAZEBOLD
        const Vector3d WORLD_GRAVITY = v3convert(world_->GetPhysicsEngine()->GetGravity().Normalize());
#else
        const Vector3d WORLD_GRAVITY = world_->Gravity().Normalize();
#endif
        // water surface is orthogonal to gravity
        surface_plane_.Set(WORLD_GRAVITY.X(), WORLD_GRAVITY.Y(), WORLD_GRAVITY.Z(), WORLD_GRAVITY.Dot(surface_point));
        // push on parameter server
        rosnode_->setParam("surface", surface_point.Z());
    }

    if(_sdf->HasElement("fluidTopic"))  fluid_topic = _sdf->Get<std::string>("fluidTopic");

    // initialize subscriber to water current
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
                fluid_topic, 1,
                boost::bind(&FreeFloatingFluidPlugin::FluidVelocityCallBack, this, _1),
                ros::VoidPtr(), &callback_queue_);
    fluid_velocity_.Set(0,0,0);
    fluid_velocity_subscriber_ = rosnode_->subscribe(ops);

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&FreeFloatingFluidPlugin::Update, this));

    // Clear existing links
    buoyant_links_.clear();
    parsed_models_.clear();

    ROS_INFO("Loaded freefloating_fluid plugin.");
}

void FreeFloatingFluidPlugin::Update()
{
    // activate callbacks
    callback_queue_.callAvailable();

    // look for new world models
    unsigned int i;
    std::vector<model_st>::iterator model_it;
    bool found;
#ifdef GAZEBOLD
    for(auto model: world_->GetModels())
#else
    for(auto model: world_->Models())
#endif
    {
        if(!model->IsStatic())
        {
            if(std::find_if(parsed_models_.begin(), parsed_models_.end(),
                            [&](const model_st &parsed){return parsed.name == model->GetName();})
                    == parsed_models_.end())    // not in parsed models
            ParseNewModel(model);
        }
    }

    // look for deleted world models
    model_it = parsed_models_.begin();
    while (model_it != parsed_models_.end())
    {
        found = false;
#ifdef GAZEBOLD
        for(i=0;i<world_->GetModelCount(); ++i)
        {
            if(world_->GetModel(i)->GetName() == model_it->name)
                found = true;
        }
#else
        for(i=0;i<world_->ModelCount(); ++i)
        {
            if(world_->ModelByIndex(i)->GetName() == model_it->name)
                found = true;
        }
#endif
        if(!found)  // model name not in world anymore, remove the corresponding links
            RemoveDeletedModel(model_it);
        else
            ++model_it;
    }

    // here buoy_links is up-to-date with the links that are subject to buoyancy, let's apply it
    Vector3d actual_force, cob_position, velocity_difference;
    double signed_distance_to_surface;
    for( auto & link:  buoyant_links_)
    {
        // get world position of the center of buoyancy
#ifdef GAZEBOLD
        cob_position = v3convert(link.link->GetWorldPose().pos +
                                 link.link->GetWorldPose().rot.RotateVector(link.buoyancy_center));
#else
        cob_position = link.link->WorldPose().Pos() +
                           link.link->WorldPose().Rot().RotateVector(link.buoyancy_center);
#endif
        // start from the theoretical buoyancy force
        actual_force = link.buoyant_force;
        if(has_surface_)
        {
            // adjust force depending on distance to surface (very simple model)
            signed_distance_to_surface = surface_plane_.W()
                    - surface_plane_.X() * cob_position.X()
                    - surface_plane_.Y() * cob_position.Y()
                    - surface_plane_.Z() * cob_position.Z();
            if(signed_distance_to_surface > -link.limit)
            {
                if(signed_distance_to_surface > link.limit)
                    actual_force *= 0;
                else
                    actual_force *= cos(M_PI/4.*(signed_distance_to_surface/link.limit + 1));
            }
        }

        // get velocity damping
        // linear velocity difference in the link frame
#ifdef GAZEBOLD
        velocity_difference = v3convert(link.link->GetWorldPose().rot.RotateVectorReverse(link.link->GetWorldLinearVel() - fluid_velocity_));
#else
        velocity_difference = link.link->WorldPose().Rot().RotateVectorReverse(link.link->WorldLinearVel() - fluid_velocity_);
#endif
        // to square
        velocity_difference.X() *= fabs(velocity_difference.X());
        velocity_difference.Y() *= fabs(velocity_difference.Y());
        velocity_difference.Z() *= fabs(velocity_difference.Z());
        // apply damping coefficients
#ifdef GAZEBOLD
        actual_force -= v3convert(link.link->GetWorldPose().rot.RotateVector(link.linear_damping * velocity_difference));
#else
        actual_force -= link.link->WorldPose().Rot().RotateVector(link.linear_damping * velocity_difference);
#endif
        link.link->AddForceAtWorldPosition(actual_force, cob_position);

        // same for angular damping
#ifdef GAZEBOLD
        velocity_difference = v3convert(link.link->GetRelativeAngularVel());
#else
        velocity_difference = link.link->RelativeAngularVel();
#endif
        velocity_difference.X() *= fabs(velocity_difference.X());
        velocity_difference.Y() *= fabs(velocity_difference.Y());
        velocity_difference.Z() *= fabs(velocity_difference.Z());
        link.link->AddRelativeTorque(-link.angular_damping*velocity_difference);

        // publish states as odometry message
        nav_msgs::Odometry state;
        state.header.frame_id = "world";
        state.header.stamp = ros::Time::now();
        Vector3d vec;
        for(const auto & model: parsed_models_)
        {
            // which link
            state.child_frame_id = "base_link";

#ifdef GAZEBOLD
            // write absolute pose            
            auto pose = model.model_ptr->GetWorldPose();
            state.pose.pose.position.x = pose.pos.x;
            state.pose.pose.position.y = pose.pos.y;
            state.pose.pose.position.z = pose.pos.z;
            state.pose.pose.orientation.x = pose.rot.x;
            state.pose.pose.orientation.y = pose.rot.y;
            state.pose.pose.orientation.z = pose.rot.z;
            state.pose.pose.orientation.w = pose.rot.w;

            // write relative linear velocity            
            vec = v3convert(model.model_ptr->GetRelativeLinearVel());
            state.twist.twist.linear.x = vec.X();
            state.twist.twist.linear.y = vec.Y();
            state.twist.twist.linear.z = vec.Z();
            // write relative angular velocity
            vec = v3convert(model.model_ptr->GetRelativeAngularVel());
            state.twist.twist.angular.x = vec.X();
            state.twist.twist.angular.y = vec.Y();
            state.twist.twist.angular.z = vec.Z();
#else
            // write absolute pose
            auto pose = model.model_ptr->WorldPose();
            state.pose.pose.position.x = pose.Pos().X();
            state.pose.pose.position.y = pose.Pos().Y();
            state.pose.pose.position.z = pose.Pos().Z();
            state.pose.pose.orientation.x = pose.Rot().X();
            state.pose.pose.orientation.y = pose.Rot().Y();
            state.pose.pose.orientation.z = pose.Rot().Z();
            state.pose.pose.orientation.w = pose.Rot().W();

            // write relative linear velocity
            vec = model.model_ptr->RelativeLinearVel();
            state.twist.twist.linear.x = vec.X();
            state.twist.twist.linear.y = vec.Y();
            state.twist.twist.linear.z = vec.Z();
            // write relative angular velocity
            vec = model.model_ptr->RelativeAngularVel();
            state.twist.twist.angular.x = vec.X();
            state.twist.twist.angular.y = vec.Y();
            state.twist.twist.angular.z = vec.Z();
#endif

            // publish
            model.state_publisher.publish(state);
        }

        //  ROS_INFO("Link %s: Applying buoyancy force (%.01f, %.01f, %.01f)", link.name.c_str(), link.buoyant_force.x, link.buoyant_force.y, link.buoyant_force.z);
    }
}

void FreeFloatingFluidPlugin::ParseNewModel(const physics::ModelPtr &_model)
{
    // define new model structure: name / pointer / publisher to odometry
    model_st new_model;
    new_model.name = _model->GetName();
    new_model.model_ptr = _model;
    new_model.state_publisher = rosnode_->advertise<nav_msgs::Odometry>("/" + _model->GetName() + "/state", 1);
    // tells this model has been parsed
    parsed_models_.push_back(new_model);

    // get robot description from model name
    // we cannot do anything without the robot_description, as a custom parsing is required to get buoyancy tags
    if(!rosnode_->hasParam("/" + _model->GetName() + "/" + description_))
        return;

    const unsigned int previous_link_number = buoyant_links_.size();
    std::string urdf_content;
    rosnode_->getParam("/" + _model->GetName() + "/" + description_, urdf_content);
    // parse actual URDF as XML (that's ugly) to get custom buoyancy tags

    // links from urdf
    TiXmlDocument urdf_doc;
    urdf_doc.Parse(urdf_content.c_str(), 0);
#ifdef GAZEBOLD
    const Vector3d WORLD_GRAVITY = v3convert(world_->GetPhysicsEngine()->GetGravity());
#else
    const Vector3d WORLD_GRAVITY = world_->Gravity();
#endif

    TiXmlElement* urdf_root = urdf_doc.FirstChildElement();
    TiXmlNode* urdf_node, *link_node, *buoy_node;
    double compensation;
    unsigned int link_index;
    physics::LinkPtr sdf_link;
    bool found;
    for(urdf_node = urdf_root->FirstChild(); urdf_node != 0; urdf_node = urdf_node->NextSibling())
    {
        if(urdf_node->ValueStr() == "link")
        {
            // find corresponding sdf model link if any
            found = false;
            for(link_index = 0; link_index < _model->GetLinks().size(); ++link_index)
            {
                if(urdf_node->ToElement()->Attribute("name") == _model->GetLinks()[link_index]->GetName())
                {
                    found = true;
                    sdf_link = _model->GetLinks()[link_index];
                    break;
                }
            }

            if(found)
            {
                for(link_node = urdf_node->FirstChild(); link_node != 0; link_node = link_node->NextSibling())
                {
                    if(link_node->ValueStr() == "buoyancy")
                    {
                        // this link is subject to buoyancy, create an instance
                        link_st new_buoy_link;
                        new_buoy_link.model_name = _model->GetName();            // in case this model is deleted
                        new_buoy_link.link =  sdf_link;    // to apply forces
                        new_buoy_link.limit = .1;

                        // get data from urdf
                        // default values
#ifdef GAZEBOLD
                        new_buoy_link.buoyancy_center = v3convert(sdf_link->GetInertial()->GetCoG());
                        new_buoy_link.linear_damping = new_buoy_link.angular_damping = 5 * Vector3d::One * sdf_link->GetInertial()->GetMass();
#else
                        new_buoy_link.buoyancy_center = sdf_link->GetInertial()->CoG();
                        new_buoy_link.linear_damping = new_buoy_link.angular_damping = 5 * Vector3d::One * sdf_link->GetInertial()->Mass();
#endif

                        compensation = 0;
                        for(buoy_node = link_node->FirstChild(); buoy_node != 0; buoy_node = buoy_node->NextSibling())
                        {
                            if(buoy_node->ValueStr() == "origin")
                                ReadVector3((buoy_node->ToElement()->Attribute("xyz")), new_buoy_link.buoyancy_center);
                            else if(buoy_node->ValueStr() == "compensation")
                                compensation = atof(buoy_node->ToElement()->GetText());
                            else if(buoy_node->ValueStr() == "limit")
                            {
                                std::stringstream ss(buoy_node->ToElement()->Attribute("radius"));
                                ss >> new_buoy_link.limit;
                            }
                            else if(buoy_node->ValueStr() == "damping")
                            {
                                if(buoy_node->ToElement()->Attribute("xyz") != NULL)
                                {
                                    ReadVector3((buoy_node->ToElement()->Attribute("xyz")), new_buoy_link.linear_damping);
                                    ROS_INFO("Found linear damping");
                                }
                                if(buoy_node->ToElement()->Attribute("rpy") != NULL)
                                {
                                    ReadVector3((buoy_node->ToElement()->Attribute("rpy")), new_buoy_link.angular_damping);
                                 ROS_INFO("Found angular damping");
                                }
                            }
                            else
                                ROS_WARN("Unknown tag <%s/> in buoyancy node for model %s", buoy_node->ValueStr().c_str(), _model->GetName().c_str());
                        }
#ifdef GAZEBOLD
                        new_buoy_link.buoyant_force = -compensation * sdf_link->GetInertial()->GetMass() * WORLD_GRAVITY;
#else
                        new_buoy_link.buoyant_force = -compensation * sdf_link->GetInertial()->Mass() * WORLD_GRAVITY;
#endif
                        // store this link
                        buoyant_links_.push_back(new_buoy_link);
                    }
                }   // out of loop: buoyancy-related nodes
            }       // out of condition: in sdf
        }           // out of loop: links
    }               // out of loop: all urdf nodes
    if(previous_link_number == buoyant_links_.size())
        ROS_INFO_NAMED("Buoyancy plugin", "No links subject to buoyancy inside %s", _model->GetName().c_str());
    else
        ROS_INFO_NAMED("Buoyancy plugin", "Added %i buoy links from %s", (int) buoyant_links_.size()-previous_link_number, _model->GetName().c_str());
}

void FreeFloatingFluidPlugin::RemoveDeletedModel(std::vector<model_st>::iterator &_model_it)
{
    ROS_INFO("Removing deleted model: %s", _model_it->name.c_str());

    // remove model stored links
    std::vector<link_st>::iterator link_it = buoyant_links_.begin();
    while (link_it != buoyant_links_.end())
    {
        if(link_it->model_name == _model_it->name)
            link_it = buoyant_links_.erase(link_it);
        else
            ++link_it;
    }
    // remove it from the list
    _model_it = parsed_models_.erase(_model_it);
}

void FreeFloatingFluidPlugin::FluidVelocityCallBack(const geometry_msgs::Vector3ConstPtr &_msg)
{
    // store fluid velocity
    fluid_velocity_.Set(_msg->x, _msg->y, _msg->z);
}

}
