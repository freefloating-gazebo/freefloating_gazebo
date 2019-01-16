#include <freefloating_gazebo/hydro_model_parser.h>
#include <urdf/model.h>
#include <Eigen/Geometry>

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

Eigen::Vector3d urdf2Eigen(const urdf::Vector3 &t)
{
  return Eigen::Vector3d(t.x, t.y, t.z);
}

Eigen::Quaterniond urdf2Eigen(const urdf::Rotation &q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}


void HydroModelParser::parseAll(ros::NodeHandle &nh, bool display)
{
  std::cout << "Parsing hydro model from namespace " << nh.getNamespace() << std::endl;
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
        parseThrusters(printer.CStr(), display?robot_namespace:"");
        break;
      }
    }
  }
  parseLinks(root, display);
}

// parse from Gazebo-passed SDF
void HydroModelParser::parseThrusters(std::string sdf_str, std::string robot_name)
{
  thrusters.clear();

  tinyxml2::XMLDocument doc;
  doc.Parse(sdf_str.c_str());
  auto sdf = doc.RootElement();

  urdf::Model model;

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
        // read URDF if needed
        if(model.getName() == "")
        {
          std::string robot_description;
          ros::NodeHandle("/" + robot_name).getParam("robot_description", robot_description);
          model.initString(robot_description);
        }
        // check if fixed thruster - only if directly attached to base_link
        if(readFixedThruster(name_info->GetText(), model, max_thrust))
        {
          if(robot_name != "")
            ROS_INFO("%s: adding %s as a fixed thruster", robot_name.c_str(), name_info->GetText());
        }
        else
        {
          // add this index to steering thrusters
          thrusters.push_back({name_info->GetText(), false, max_thrust, {}});
          if(robot_name != "")
            ROS_INFO("%s: adding %s as a steering thruster", robot_name.c_str(), name_info->GetText());
        }
      }
    }
  }
}

bool HydroModelParser::readFixedThruster(std::string thruster_name, const urdf::Model &model, double max_thrust)
{
  // get thruster link
  auto link = model.getLink(thruster_name);

  // find transform to base_link
  Eigen::Quaterniond q(1,0,0,0);
  Eigen::Vector3d t(0,0,0);

  while(link->name != "base_link")
  {
    const auto joint = link->parent_joint;
    link = link->getParent();

    // check joint type
    if(joint->type != urdf::Joint::FIXED)
      return false;

    // read this transform
    const auto pose = joint->parent_to_joint_origin_transform;
    const auto q_joint(urdf2Eigen(pose.rotation));
    const auto t_joint(urdf2Eigen(pose.position));

    // update global transform
    q = q_joint * q;
    t = t_joint + q_joint*t;
  }
  // build full map of fixed thruster
  // thrust direction in robot frame
  const Eigen::Vector3d thrust = q*Eigen::Vector3d(0,0,-1);
  thrusters.push_back({thruster_name, true, max_thrust,
                       {thrust.x(),
                        thrust.y(),
                        thrust.z(),
                        t.y()*thrust.z() - t.z()*thrust.y(),
                        t.z()*thrust.x() - t.x()*thrust.z(),
                        t.x()*thrust.y() - t.y()*thrust.x()}});
  return true;
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
    const std::string name(link->Attribute("name"));
    // check in URDF if root link
    const auto parent_joint = model.getLink(name)->parent_joint;
    if(parent_joint == nullptr || parent_joint->type != urdf::Joint::FIXED)
      links[name] = parseLink(link, model);
  }
}


HydroLink HydroModelParser::parseLink(const tinyxml2::XMLElement* elem, const urdf::Model &model, bool is_root)
{
  const auto name = elem->Attribute("name");
  HydroLink link;

  // write inertia part at CoG
  const auto urdf_link = model.getLink(name);
  if(urdf_link->inertial != nullptr)
  {
    link.mass = urdf_link->inertial->mass;
    link.cog = urdf2Eigen(urdf_link->inertial->origin.position);
    // inertia matrix written in CoG frame
    link.inertia.block<3,3>(0,0) = link.mass * Eigen::Matrix3d::Identity();
    link.inertia(3,3) = urdf_link->inertial->ixx;
    link.inertia(4,4) = urdf_link->inertial->iyy;
    link.inertia(5,5) = urdf_link->inertial->izz;
    link.inertia(3,4) = link.inertia(4,3) = urdf_link->inertial->ixy;
    link.inertia(3,5) = link.inertia(5,3) = urdf_link->inertial->ixz;
    link.inertia(4,5) = link.inertia(5,4) = urdf_link->inertial->iyz;
    // write in link frame
    const auto q = urdf2Eigen(urdf_link->inertial->origin.rotation);
    link.inertia.block<3,3>(3,3) = q * link.inertia.block<3,3>(3,3) * q.inverse();
  }

  auto tag = elem->FirstChildElement("buoyancy");
  if(tag)
  {
    // old-style model: everything in buoyancy tag
    const auto density = readDensity(tag);    
    const auto node = tag->FirstChildElement("origin");
    if(node)
      link.cob = readVector3(node->Attribute("xyz"));
    addBuoyancy(link, tag, model.getLink(name)->inertial, density);
    addHydrodynamics(link, tag, density);
  }

  tag = elem->FirstChildElement("fluid");
  if(tag)
  {
    // new-style - in fluid
    const auto density = readDensity(tag);
    const auto node = tag->FirstChildElement("origin");
    if(node)
      link.cob = readVector3(node->Attribute("xyz"));

    addBuoyancy(link, tag->FirstChildElement("buoyancy"),
                urdf_link->inertial, density);
    addHydrodynamics(link, tag->FirstChildElement("hydrodynamics"), density);
  }

  // find fixed child links
  HydroLink sub_link;
  for(const auto &child: model.getLink(name)->child_links)
  {
    if(child->parent_joint->type == urdf::Joint::FIXED)
    {

      // find corresponding element in raw XML
      for(auto subelem = elem->Parent()->FirstChildElement("link"); subelem != nullptr;
          subelem = subelem->NextSiblingElement("link"))
      {
        if(child->name.compare(subelem->Attribute("name")) == 0)
        {
          sub_link = parseLink(subelem, model, false);
          break;
        }
      }
      const auto pose = child->parent_joint->parent_to_joint_origin_transform;
      const auto q = urdf2Eigen(pose.rotation);
      const auto t = urdf2Eigen(pose.position);

      // Update inertia parameters wrt new CoG in link frame
      // write CoG of sublink in this frame
      sub_link.cog = t + q*sub_link.cog;
      // new CoG
      const auto cog = (link.mass*link.cog + sub_link.mass*sub_link.cog)/
          (link.mass + sub_link.mass);
      // mass update
      for(int i = 0; i < 3; ++i)
        link.inertia(i,i) += sub_link.mass;
      // inertia of this link wrt new CoG
      link.inertia.block<3,3>(3,3) -= link.mass * skew(cog - link.cog) * skew(cog - link.cog);
      // add inertia of sublink wrt new CoG
      link.inertia.block<3,3>(3,3) += q * sub_link.inertia.block<3,3>(3,3) * q.inverse()
          - sub_link.mass * skew(cog - sub_link.cog) * skew(cog - sub_link.cog);
      // erase mass and CoG
      link.mass += sub_link.mass;
      link.cog = cog;

      // TODO update buoyancy and hydrodynamics from fixed child links
    }
  }

  if(is_root)
  {
    // write inertia at link origin
    link.inertia.block<3,3>(3,3) -= - link.mass * skew(link.cog) * skew(link.cog);
    link.inertia.block<3,3>(0,3) = -link.mass * skew(link.cog);
    link.inertia.block<3,3>(3,0) = link.mass * skew(link.cog);

    // TODO write hydrodynamics at link origin (written at CoB)


  }
  return link;
}


void HydroModelParser::addBuoyancy(HydroLink &link, const tinyxml2::XMLElement* elem, const urdf::InertialSharedPtr &inertial, double density)
{
  // no hydro tag or no mass -> no buoyancy (will not be simulated by Gazebo anyway)
  if(elem == nullptr)
    return;
  if(inertial == nullptr)
    return;

  for(auto node = elem->FirstChildElement(); node != nullptr; node = node->NextSiblingElement())
  {
    if(strcmp(node->Value(), "compensation") == 0)
      link.buoyancy_force = 9.81 * inertial->mass * atof(node->GetText()) * density;
    else if(strcmp(node->Value(), "limit") == 0)
      link.buoyancy_limit = atof(node->Attribute("radius"));
  }
}


void HydroModelParser::addHydrodynamics(HydroLink &link, const tinyxml2::XMLElement* elem, double density)
{
  if(elem == nullptr)
    return;

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
  // density factor
  if(link.has_added_mass)
    link.added_mass *= density;
  if(link.has_lin_damping)
    link.lin_damping *= density;
  if(link.has_quad_damping)
    link.quad_damping *= density;
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
  const uint n =  static_cast<uint>(thrusters.size());
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
  std::vector<double> max_vel(6,0);
  const auto &base_link = links.at("base_link");
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
  }
  return max_vel;
}

}
