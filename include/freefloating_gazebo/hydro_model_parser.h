#ifndef HYDRO_MODEL_PARSER_H
#define HYDRO_MODEL_PARSER_H


#include <tinyxml2.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <freefloating_gazebo/hydro_link.h>
#include <urdf/model.h>

namespace ffg
{

class HydroModelParser
{
  struct Thruster
  {
    std::string name;
    bool fixed;
    double max_thrust;
    std::vector<double> map;
  };



public:
  HydroModelParser() {}

  void parseAll(ros::NodeHandle &nh, bool display = true);
  void parseThrusters(std::string sdf_str, std::string robot_name);
  bool readFixedThruster(std::string thruster_name, const urdf::Model &model, double max_thrust);
  void parseLinks(ros::NodeHandle &nh, bool display = true)
  {
    std::string robot_description;
    nh.getParam("robot_description", robot_description);
    parseLinks(robot_description, display);
  }
  void parseLinks(const std::string &robot_description, bool display = true)
  {
    tinyxml2::XMLDocument doc;
    doc.Parse(robot_description.c_str());
    parseLinks(doc.RootElement(), display);
  }
  void parseLinks(tinyxml2::XMLElement *root, bool display = true);

  HydroLink getLink(std::string name) const {return links.at(name);}
  std::map<std::string, HydroLink> getLinks() const {return links;}
  std::vector<Thruster> getThrusters() const {return thrusters;}
  Eigen::MatrixXd thrusterMap() const;

  // utility functions for base_link
  uint thrusterInfo(std::vector<uint> &thruster_fixed,
                    std::vector<uint> &thruster_steering,
                    std::vector<std::string> &thruster_names,
                    std::vector<double> &max_thrust) const;
  std::vector<double> maxVelocity() const;
  std::vector<double> maxWrench() const;

protected:
  std::map<std::string, HydroLink> links;
  std::vector<Thruster> thrusters;
  std::string base_link = "base_link";

  bool isNamed(tinyxml2::XMLElement* elem, std::string name) const
  {
    return strcmp(elem->Attribute("name"), name.c_str()) == 0;
  }

  double readDensity(const tinyxml2::XMLElement* elem) const
  {
    const auto node = elem->FirstChildElement("density");
    if(node)
      return atof(node->GetText());
    return 1;
  }

  HydroLink parseLink(const tinyxml2::XMLElement* elem, const urdf::Model &model, bool is_root = true);
  void addBuoyancy(HydroLink &link, const tinyxml2::XMLElement* elem, const urdf::InertialSharedPtr &inertial, double density);
  void addHydrodynamics(HydroLink &link, const tinyxml2::XMLElement* elem, double density);

};

}

#endif // HYDRO_MODEL_PARSER_H
