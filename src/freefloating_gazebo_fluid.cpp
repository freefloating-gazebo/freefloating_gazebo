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

#include <freefloating_gazebo/freefloating_gazebo_fluid.h>
#include <freefloating_gazebo/hydro_model_parser.h>

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
 #ifdef GAZEBOLD
  world_->GetPhysicsEngine()->GetUpdatePeriod();
#else
   world_->Physics()->GetUpdatePeriod();
#endif
  // register ROS node
  rosnode_ = new ros::NodeHandle("gazebo");

  // parse plugin options
  has_surface_ = false;
  surface_plane_.Set(0,0,1,0); // default ocean surface plane is Z=0
  std::string fluid_topic = "current";

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
#ifdef GAZEBOLD
  for(const auto &model: world_->GetModels())
#else
  for(const auto &model: world_->Models())
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
  auto model = parsed_models_.begin();
  while(model != parsed_models_.end())
  {
    bool still_here = false;
#ifdef GAZEBOLD
    for(uint i=0;i<world_->GetModelCount(); ++i)
    {
      if(world_->GetModel(i)->GetName() == model->name)
      {
        still_here = true;
        break;
      }
    }
#else
    for(uint i=0;i<world_->ModelCount(); ++i)
    {
      if(world_->ModelByIndex(i)->GetName() == model->name)
      {
        still_here = true;
        break;
      }
    }
#endif
    if(!still_here)
      RemoveModel(model);
    else
      model++;
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
    Vector3d force, torque;
    if(has_surface_)
    {
      // adjust force depending on distance to surface (very simple model)
      signed_distance_to_surface = surface_plane_.W()
          - surface_plane_.X() * cob_position.X()
          - surface_plane_.Y() * cob_position.Y()
          - surface_plane_.Z() * cob_position.Z();
      force = eigen2Gazebo(link.hydro.buoyancyForce(signed_distance_to_surface));
    }

    // velocity difference in the link frame
     Eigen::Vector6d velocity;
#ifdef GAZEBOLD
    velocity.head<3>() = gazebo2Eigen(v3convert(link.link->GetWorldPose().rot.RotateVectorReverse(link.link->GetWorldLinearVel() - fluid_velocity_)));
    velocity.tail<3>() = gazebo2Eigen(v3convert(link.link->GetRelativeAngularVel()));
#else
    velocity.head<3>() = gazebo2Eigen(link.link->WorldPose().Rot().RotateVectorReverse(link.link->WorldLinearVel() - fluid_velocity_));
    velocity.tail<3>() = gazebo2Eigen(link.link->RelativeAngularVel());
#endif

    // resulting force
    const auto dyn_force = link.hydro.hydroDynamicForce(velocity);

    // apply force
#ifdef GAZEBOLD
    link.link->AddForceAtWorldPosition(force + v3convert(link.link->GetWorldPose().rot.RotateVector(eigen2Gazebo(dyn_force.head<3>()))),
                                       cob_position);
#else
    link.link->AddForceAtWorldPosition(force + link.link->WorldPose().Rot().RotateVector(eigen2Gazebo(dyn_force.head<3>())),
                                       cob_position);
#endif
    // apply torque
    link.link->AddRelativeTorque(eigen2Gazebo(dyn_force.tail<3>()));

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
  ROS_INFO("Parsing hydro for model '%s'...", _model->GetName().c_str());

  // define new model structure: name / pointer / publisher to odometry
  model_st new_model;
  new_model.name = _model->GetName();
  new_model.model_ptr = _model;
  new_model.state_publisher = rosnode_->advertise<nav_msgs::Odometry>("/" + _model->GetName() + "/state", 1);
  // tells this model has been parsed
  parsed_models_.push_back(new_model);

  // get link properties from robot_description to add hydro effects
  if(!rosnode_->hasParam("/" + _model->GetName() + "/robot_description"))
    return;

  const auto previous_link_number = buoyant_links_.size();
  std::string robot_description;
  rosnode_->getParam("/" + _model->GetName() + "/robot_description", robot_description);
  ffg::HydroModelParser parser;
  parser.parseLinks(robot_description);

  for(auto &link: parser.getLinks())
  {
    // find corresponding sdf model link if any
    const auto sdf_link = std::find_if(_model->GetLinks().begin(),
                                       _model->GetLinks().end(),
                                       [&](const physics::LinkPtr &linkptr){return linkptr->GetName() == link.first;});

    if(sdf_link != _model->GetLinks().end())
    {
      // this link is subject to buoyancy, create an instance
      buoyant_links_.push_back({_model->GetName(), *sdf_link, eigen2Gazebo(link.second.cob), link.second});
#ifdef GAZEBOLD
      buoyant_links_.back().hydro.initFilters(world_->GetPhysicsEngine()->GetUpdatePeriod());
#else
      buoyant_links_.back().hydro.initFilters(world_->Physics()->GetUpdatePeriod());
#endif
      }
  }

  if(previous_link_number == buoyant_links_.size())
    ROS_INFO_NAMED("Buoyancy plugin", "No links subject to buoyancy inside %s", _model->GetName().c_str());
  else
    ROS_INFO_NAMED("Buoyancy plugin", "Added %i buoy links from %s", static_cast<int>(buoyant_links_.size()-previous_link_number), _model->GetName().c_str());
}

void FreeFloatingFluidPlugin::RemoveModel(std::vector<model_st>::iterator &model)
{
  ROS_INFO("Removing deleted model: %s", model->name.c_str());

  // remove model stored links
  buoyant_links_.erase(std::remove_if(buoyant_links_.begin(),
                                      buoyant_links_.end(),
                                      [&](link_st & link)
  {return link.model_name == model->name;}), buoyant_links_.end());

  // remove it from the list
  model = parsed_models_.erase(model);
}

void FreeFloatingFluidPlugin::FluidVelocityCallBack(const geometry_msgs::Vector3ConstPtr &_msg)
{
  // store fluid velocity
  fluid_velocity_.Set(_msg->x, _msg->y, _msg->z);
}

}
