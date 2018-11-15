#include <freefloating_gazebo/thruster_mapper.h>

namespace ffg
{

// parse from robot_description param
void ThrusterMapper::parse(ros::NodeHandle &nh, bool display)
{
  std::string s;
  nh.getParam("robot_description", s);
  TiXmlDocument doc;
  doc.Parse(s.c_str());
  auto root = doc.RootElement();
  base_link = "base_link";

  // find ffg plugin part
  for(auto elem = root->FirstChildElement("gazebo"); elem != nullptr; elem = elem->NextSiblingElement("gazebo"))
  {
    // check for plugin
    TiXmlElement* plugin = elem->FirstChildElement("plugin");
    if(plugin != nullptr)
    {
      if(checkName(plugin, "freefloating_gazebo_control"))
      {
        TiXmlPrinter printer;
        plugin->Accept(&printer);
        // parse sdf
        parse(printer.CStr(), display);

        // also get base link name
        if(plugin->FirstChildElement("link"))
          base_link = plugin->FirstChildElement("link")->GetText();
        break;
      }
    }
  }
}

// parse from Gazebo-passed SDF
void ThrusterMapper::parse(std::string sdf_str, bool display)
{
  names.clear();
  fixed_idx.clear();
  steer_idx.clear();
  max_command.clear();

  TiXmlDocument doc;
  doc.Parse(sdf_str.c_str());
  auto sdf = doc.RootElement();

  double map_coef;
  auto sdf_element = sdf->FirstChildElement();
  while(sdf_element)
  {
    if(sdf_element->ValueStr() == "thruster")
    {
      // check if it is a steering or fixed thruster
      auto map_info = sdf_element->FirstChildElement("map");
      if(map_info)  // fixed thruster
      {
        // add this index to fixed thrusters
        fixed_idx.push_back(names.size());

        // register map coefs
        std::stringstream ss(map_info->GetText());
        const auto nb = map.cols();
        map.conservativeResize(6,nb+1);
        for(int i=0;i<6;++i)
        {
          ss >> map_coef;
          map(i,nb) = map_coef;
        }

        // check for any name
        auto name_info = sdf_element->FirstChildElement("name");
        if(name_info)
          names.push_back(std::string(name_info->GetText()));
        else
        {
          std::ostringstream name;
          name << "thr" << names.size();
          names.push_back(name.str());
        }

        if(display)
          ROS_INFO("Adding %s as a fixed thruster", names[names.size()-1].c_str());
      }
      else
      {
        auto name_info = sdf_element->FirstChildElement("name");
        if(name_info)
        {
          // add this index to steering thrusters
          steer_idx.push_back(names.size());

          // add the link name
          names.push_back(name_info->GetText());
          if(display)
            ROS_INFO("Adding %s as a steering thruster", names[names.size()-1].c_str());
        }
      }

      // register maximum effort
      auto effort_info = sdf_element->FirstChildElement("effort");
      if(effort_info)
      {
        std::stringstream ss(effort_info->GetText());
        double val;
        ss >> val;
        max_command.push_back(val);
      }
      else
        max_command.push_back(100);
    }
    sdf_element = sdf_element->NextSiblingElement();
  }
}

std::vector<std::string> ThrusterMapper::initControl(ros::NodeHandle &nh, bool compute_inverse_map)
{
  std::string s;
  nh.getParam("robot_description", s);

  TiXmlDocument doc;
  doc.Parse(s.c_str());
  auto root = doc.RootElement();

  // find max velocity on each axis
  max_vel.resize(6, 0);
  damping.resize(6, 0);
  std::vector<std::string> controlled_axes;
  const std::vector<std::string> axes{"x", "y", "z", "roll", "pitch", "yaw"};

  for(auto elem = root->FirstChildElement("link"); elem != nullptr; elem = elem->NextSiblingElement("link"))
  {
    if(checkName(elem, base_link))
    {
      auto buoy = elem->FirstChildElement("buoyancy");
      if(buoy)
      {
        auto damp = buoy->FirstChildElement("damping");
        // get damping

        for(TiXmlAttribute* att = damp->FirstAttribute(); att != nullptr; att = att->Next())
        {
          if(att->NameTStr() == "xyz")
          {
            std::stringstream ss(att->Value());
            ss >> damping[0];
            ss >> damping[1];
            ss >> damping[2];
          }
          else if(att->NameTStr() == "rpy")
          {
            std::stringstream ss(att->Value());
            ss >> damping[3];
            ss >> damping[4];
            ss >> damping[5];
          }
        }
        for(size_t ax = 0; ax < 6; ++ax)
        {
          // get max effort in this direction and init PID clamp
          double max_thrust = 0;
          for(uint j = 0; j < map.cols(); j++)
            max_thrust += std::abs(map(static_cast<Eigen::Index>(ax), j)) * max_command[j];

          if(std::abs(max_thrust) > 1e-6)
          {
            char param[FILENAME_MAX];
            sprintf(param, "controllers/%s", axes[ax].c_str());
            if(nh.hasParam(param))
            {
              controlled_axes.push_back(axes[ax]);
              // push to position PID
              sprintf(param, "controllers/%s/position/i_clamp", axes[ax].c_str());
              nh.setParam(param, max_thrust);
              // push to velocity PID
              sprintf(param, "controllers/%s/velocity/i_clamp", axes[ax].c_str());
              nh.setParam(param, max_thrust);
            }
          }
          if(std::abs(damping[ax]) > 1e-6)
          {
            // save max velocity on this axis
            max_vel[ax] = max_thrust / damping[ax];
          }
        }
      }
      break;
    }
  }

  if(compute_inverse_map)
  {
    inverse_map.resize(map.cols(), map.rows());
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_M(map, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd dummy_in;
    Eigen::VectorXd dummy_out(map.cols());
    unsigned int i,j;
    for(i=0;i<map.rows();++i)
    {
      dummy_in = Eigen::VectorXd::Zero(map.rows());
      dummy_in(i) = 1;
      dummy_out = svd_M.solve(dummy_in);
      for(j = 0; j<map.cols();++j)
        inverse_map(j,i) = dummy_out(j);
    }
  }
  return controlled_axes;
}

void ThrusterMapper::saturate(Eigen::VectorXd &_command) const
{
  double norm_ratio = 1;
  unsigned int i;
  for(i=0;i<fixed_idx.size();++i)
    norm_ratio = std::max(norm_ratio, std::abs(_command(i)) / max_command[i]);
  _command *= 1./norm_ratio;
}

sensor_msgs::JointState ThrusterMapper::wrench2Thrusters(const geometry_msgs::Wrench &cmd) const
{
  Eigen::VectorXd wrench(6);
  wrench << cmd.force.x, cmd.force.y, cmd.force.z,
      cmd.torque.x, cmd.torque.y, cmd.torque.z;
  sensor_msgs::JointState msg;
  msg.name = names;
  msg.effort.reserve(names.size());

  Eigen::VectorXd thrust = inverse_map * wrench;
  saturate(thrust);

  for(int i = 0; i < thrust.rows(); i++)
  {
    msg.effort.push_back(thrust[i]);
  }
  return msg;
}
}
