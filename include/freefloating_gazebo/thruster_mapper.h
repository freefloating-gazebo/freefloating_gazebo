#ifndef MAPPER_H
#define MAPPER_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <tinyxml.h>

namespace ffg
{

class ThrusterMapper
{
public:
    ThrusterMapper()  {}

    bool checkName(TiXmlElement* elem, std::string name)
    {
        for(TiXmlAttribute* att = elem->FirstAttribute(); att != NULL; att = att->Next())
        {
            if(att->NameTStr() == "name" && att->ValueStr() == name)
                return true;
        }
        return false;
    }

    // parse raw param to get thruster max force and map
    // only for fixed thrusters (for now)
    void parse(ros::NodeHandle &nh, bool display = true);

    // parse for Gazebo
    void parse(std::string sdf_str, bool display = true);

    bool has_thrusters() const {return names.size();}

    std::vector<std::string> initControl(ros::NodeHandle &nh, bool compute_inverse_map = true);


    geometry_msgs::Wrench maxWrench() const
    {
      geometry_msgs::Wrench max_wrench;
      max_wrench.force.x = max_vel[0] * damping[0];
      max_wrench.force.y = max_vel[1] * damping[1];
      max_wrench.force.z = max_vel[2] * damping[2];
      max_wrench.torque.x = max_vel[3] * damping[3];
      max_wrench.torque.y = max_vel[4] * damping[4];
      max_wrench.torque.z = max_vel[5] * damping[5];
      return max_wrench;
    }

    void saturate(Eigen::VectorXd &_command) const;

    sensor_msgs::JointState wrench2Thrusters(const geometry_msgs::Wrench  & cmd) const;

    std::string base_link;
    std::vector<size_t> steer_idx, fixed_idx;
    std::vector<std::string> names;
    std::vector<double> max_command, max_vel, damping;

    Eigen::MatrixXd map;
    Eigen::MatrixXd inverse_map;
};

}



#endif // MAPPER_H
