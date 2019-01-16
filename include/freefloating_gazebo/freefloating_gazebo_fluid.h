#ifndef FREEFLOATINGGAZEBOFLUID_H
#define FREEFLOATINGGAZEBOFLUID_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <freefloating_gazebo/hydro_link.h>

namespace gazebo
{

class FreeFloatingFluidPlugin : public  WorldPlugin
{

    typedef ignition::math::Vector3d Vector3d;


public:
    FreeFloatingFluidPlugin() {}
    ~FreeFloatingFluidPlugin()
    {
        rosnode_->shutdown();
        delete rosnode_;
    }

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    struct link_st
    {
        std::string model_name;
        physics::LinkPtr link;
        Vector3d buoyancy_center;
        ffg::HydroLink hydro;
    };

    struct model_st
    {
        std::string name;
        physics::ModelPtr model_ptr;
        ros::Publisher state_publisher;
    };

    Vector3d eigen2Gazebo(const Eigen::Vector3d &in)
    {
      return Vector3d(in.x(), in.y(), in.z());
    }

    Eigen::Vector3d gazebo2Eigen(const Vector3d &in)
    {
      return Eigen::Vector3d(in.X(), in.Y(), in.Z());
    }

    // parse a Vector3 string
    void ReadVector3(const std::string &_string, Vector3d &_vector);
    // parse a new model
    void ParseNewModel(const physics::ModelPtr &_model);
    // removes a deleted model
    void RemoveModel(std::vector<model_st>::iterator &model);
    // parse received fluid velocity message
    void FluidVelocityCallBack(const geometry_msgs::Vector3ConstPtr& _msg);

private:
    // plugin options
    bool has_surface_;
    ignition::math::Vector4d surface_plane_;

    // general data
    ros::NodeHandle* rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_event_;

    // links that are subject to fluid effects
    std::vector<link_st> buoyant_links_;
    // models that have been parsed
    std::vector<model_st> parsed_models_;

    // subscriber to fluid velocity (defined in the world frame)
    ros::Subscriber fluid_velocity_subscriber_;
    Vector3d fluid_velocity_;

};
GZ_REGISTER_WORLD_PLUGIN(FreeFloatingFluidPlugin)
}
#endif // FREEFLOATINGGAZEBOFLUID_H
