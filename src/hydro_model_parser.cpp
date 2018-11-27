#include <freefloating_gazebo/hydro_model_parser.h>
#include <urdf/model.h>

namespace ffg
{


Eigen::Vector3d readVector3(const std::string &_string)
{
  Eigen::Vector3d vec;
  std::stringstream ss(_string);
  for(unsigned int i=0;i<3;++i)
    ss >> vec[i];
  return vec;
}

void HydroModelParser::parseAll(ros::NodeHandle &nh, bool display)
{
  std::string robot_description;
  nh.getParam("robot_description", robot_description);
  tinyxml2::XMLDocument doc;
  doc.Parse(robot_description.c_str());
  auto root = doc.RootElement();
  const std::string robot_namespace = nh.getNamespace();

  // Thruster part - look for ffg plugin to get thruster list
  for(auto elem = root->FirstChildElement("gazebo"); elem != nullptr; elem = elem->NextSiblingElement("gazebo"))
  {
    // check for plugin
    const auto plugin = elem->FirstChildElement("plugin");
    if(plugin != nullptr)
    {
      if(isNamed(plugin, "freefloating_gazebo_control"))
      {
        tinyxml2::XMLPrinter printer;
        plugin->Accept(&printer);
        // parse sdf
        parseThusters(printer.CStr(), display?robot_namespace:"");
        break;
      }
    }
  }
  parseLinks(root, display);
}

// parse from Gazebo-passed SDF
void HydroModelParser::parseThusters(std::string sdf_str, std::string robot_name)
{
  thrusters.clear();

  tinyxml2::XMLDocument doc;
  doc.Parse(sdf_str.c_str());
  auto sdf = doc.RootElement();

  for(auto sdf_element = sdf->FirstChildElement("thruster"); sdf_element != nullptr; sdf_element = sdf_element->NextSiblingElement("thruster"))
  {
    // get max thrust anyway
    const auto effort_info = sdf_element->FirstChildElement("effort");
    double max_thrust = 100;
    if(effort_info)
    {
      std::stringstream ss(effort_info->GetText());
      ss >> max_thrust;
    }

    // check if it is a steering or fixed thruster
    const auto map_info = sdf_element->FirstChildElement("map");

    if(map_info)  // fixed thruster without link
    {
      // check for any name
      const auto name_info = sdf_element->FirstChildElement("name");
      std::string name;
      if(name_info)
        name = std::string(name_info->GetText());
      else
      {
        std::ostringstream ss;
        ss << "thr" << thrusters.size();
        name = ss.str();;
      }

      // new thruster
      thrusters.push_back({name, true, max_thrust, {0,0,0,0,0,0}});

      // register map coefs
      std::stringstream ss(map_info->GetText());
      for(auto &v: thrusters.back().map)
        ss >> v;

      if(robot_name != "")
        ROS_INFO("%s: adding %s as a fixed thruster", robot_name.c_str(), thrusters.back().name.c_str());
    }
    else
    {
      auto name_info = sdf_element->FirstChildElement("name");
      if(name_info)
      {
        // TODO check if this thruster link is actually fixed wrt base_link
        // add this index to steering thrusters
        thrusters.push_back({name_info->GetText(), false, max_thrust, {}});

        if(robot_name != "")
          ROS_INFO("%s: adding %s as a steering thruster", robot_name.c_str(), name_info->GetText());
      }
    }
  }
}

void HydroModelParser::parseLinks(tinyxml2::XMLElement *root, bool display)
{
  // TODO regroup properties of rigidly linked links into the root one for Gazebo compat
  urdf::Model model;
  // build urdf from string for TinyXLM1 compat
  tinyxml2::XMLPrinter printer;
  root->Accept(&printer);
  model.initString(printer.CStr());

  for(auto link = root->FirstChildElement("link"); link != nullptr; link = link->NextSiblingElement("link"))
  {
    const std::string name(link->ToElement()->Attribute("name"));
    auto tag = link->FirstChildElement("buoyancy");
    if(tag)
    {
      // old-style model: everything in buoyancy tag
      addBuoyancy(tag, name, model.getLink(name)->inertial);
      addHydrodynamics(tag, name);
    }

    tag = link->FirstChildElement("fluid");
    if(tag)
    {
      // new-style - in fluid
      addBuoyancy(tag->FirstChildElement("buoyancy"), name, model.getLink(name)->inertial);
      addHydrodynamics(tag->FirstChildElement("hydrodynamics"), name);
    }
  }
}


void HydroModelParser::addBuoyancy(const tinyxml2::XMLElement* elem, const std::string &name, const urdf::InertialSharedPtr &inertial)
{
  // no hydro tag or no mass -> no buoyancy (will not be simulated by Gazebo anyway)
  if(elem == nullptr)
    return;
  if(inertial == nullptr)
    return;

  // get this link or create a new one if needed
  HydroLink &link = links[name];

  for(auto node = elem->FirstChild(); node != nullptr; node = node->NextSibling())
  {
    if(strcmp(node->Value(), "origin") == 0)
      link.buoyancy_center = readVector3(node->ToElement()->Attribute("xyz"));
    else if(strcmp(node->Value(), "compensation") == 0)
      link.buoyancy_force = 9.81 * inertial->mass * atof(node->ToElement()->GetText());
    else if(strcmp(node->Value(), "limit") == 0)
      link.buoyancy_limit = atof(node->ToElement()->Attribute("radius"));
  }
}


void HydroModelParser::addHydrodynamics(const tinyxml2::XMLElement* elem, const std::string &name)
{
  if(elem == nullptr)
    return;

  // get this link
  HydroLink &link = links[name];

  for(auto node = elem->FirstChildElement(); node != nullptr; node = node->NextSiblingElement())
  {
    if(strcmp(node->Value(), "added_mass") == 0)
    {
      std::stringstream ss(node->GetText());
      link.added_mass.resize(6, 6);
      link.has_added_mass = true;
      for(uint i = 0; i < 6; ++i)
      {
        for(uint j = 0; j < 6; ++j)
          ss >> link.added_mass(i,j);
      }
    }

    else if(strcmp(node->Value(), "damping") == 0)
    {
      bool linear = false;
      if(node->Attribute("type") != nullptr)
        linear = strcmp(node->Attribute("type"), "linear") == 0;

      if(node->Attribute("xyz") != nullptr)
      {
        if(linear)
        {
          link.has_lin_damping = true;
          link.lin_damping.head<3>() = readVector3(node->Attribute("xyz"));
          ROS_INFO("Found linear linear damping");
        }
        else
        {
          link.has_quad_damping = true;
          link.quad_damping.head<3>() = readVector3(node->Attribute("xyz"));
          ROS_INFO("Found quadratic linear damping");
        }
      }

      if(node->Attribute("rpy") != nullptr)
      {
        if(linear)
        {
          link.has_lin_damping = true;
          link.lin_damping.tail<3>() = readVector3(node->Attribute("rpy"));
          ROS_INFO("Found linear angular damping");
        }
        else
        {
          link.has_quad_damping = true;
          link.quad_damping.tail<3>() = readVector3(node->Attribute("rpy"));
          ROS_INFO("Found quadratic angular damping");
        }
      }
    }
  }
}

Eigen::MatrixXd HydroModelParser::thrusterMap() const
{
  const auto n = std::count_if(thrusters.begin(),
                               thrusters.end(),
                               [](const Thruster &thr){return thr.fixed;});
  Eigen::MatrixXd map(6, n);
  uint thr_idx = 0;
  for(const auto & thruster: thrusters)
  {
    if(thruster.fixed)
    {
      for(uint axis = 0; axis < 6; ++axis)
        map(axis, thr_idx) = thruster.map[axis];
      thr_idx++;
    }
  }
  return map;
}

// utility functions for base_link
uint HydroModelParser::thrusterInfo(std::vector<uint> &thruster_fixed,
                                    std::vector<uint> &thruster_steering,
                                    std::vector<std::string> &thruster_names,
                                    std::vector<double> &thruster_max) const
{
  const uint n = thrusters.size();
  thruster_fixed.reserve(n);
  thruster_fixed.reserve(n);
  thruster_steering.reserve(n);
  thruster_names.reserve(n);
  thruster_max.reserve(n);
  uint i = 0;
  for(const auto &thruster: thrusters)
  {
    thruster_names.push_back(thruster.name);
    thruster_max.push_back(thruster.max_thrust);
    if(thruster.fixed)
      thruster_fixed.push_back(i);
    else
      thruster_steering.push_back(i);
    i++;
  }
  return n;
}

std::vector<double> HydroModelParser::maxWrench() const
{
  std::vector<double> wrench(6, 0);
  for(const auto & thruster: thrusters)
  {
    if(thruster.fixed)
    {
      for(size_t i = 0; i < 6; ++i)
        wrench[i] += thruster.max_thrust * std::abs(thruster.map[i]);
    }
  }
  return wrench;
}

std::vector<double> HydroModelParser::maxVelocity() const
{
  const auto wrench = maxWrench();
  std::vector<double> max_vel(6);
  const auto &base_link = links.find("base_link")->second;
  for(uint axis = 0; axis < 6; ++axis)
  {
    const double lin = base_link.has_lin_damping?base_link.lin_damping[axis]:0;
    const double quad = base_link.has_quad_damping?base_link.quad_damping[axis]:0;

    if(quad > 1e-6)
    {
      const double D = lin*lin + 4*quad*wrench[axis];
      max_vel[axis] = (-lin + sqrt(D))/(2*quad);
    }
    else if(lin > 1e-6)
      max_vel[axis] = wrench[axis] / lin;
    else
      max_vel[axis] = 0;
  }
  return max_vel;
}


}
