#ifndef MAPPER_H
#define MAPPER_H

#include <sdf/Element.hh>
#include <Eigen/Core>
#include <ros/ros.h>

namespace ffg
{

class ThrusterMapper
{
public:
    ThrusterMapper()
    {

    }



    void parse(sdf::ElementPtr &_sdf)
    {
        names.clear();
        fixed_idx.clear();
        steer_idx.clear();
        max_command.clear();

        double map_coef;
        sdf::ElementPtr sdf_element = _sdf->GetFirstElement();
        while(sdf_element)
        {
            if(sdf_element->GetName() == "thruster")
            {
                // check if it is a steering or fixed thruster
                if(sdf_element->HasElement("map"))  // fixed
                {
                    // add this index to fixed thrusters
                    fixed_idx.push_back(names.size());

                    // register map coefs
                    std::stringstream ss(sdf_element->Get<std::string>("map"));
                    const unsigned int nb = map.cols();
                    map.conservativeResize(6,nb+1);
                    for(int i=0;i<6;++i)
                    {
                        ss >> map_coef;
                        map(i,nb) = map_coef;
                    }

                    // check for any name
                    if(sdf_element->HasElement("name"))
                        names.push_back(sdf_element->Get<std::string>("name"));
                    else
                    {
                        std::ostringstream name;
                        name << "thr" << names.size();
                        names.push_back(name.str());
                    }

                    ROS_INFO("Adding %s as a fixed thruster", names[names.size()-1].c_str());
                }
                else if(sdf_element->HasElement("name"))
                {
                    // add this index to steering thrusters
                    steer_idx.push_back(names.size());

                    // add the link name
                    names.push_back(sdf_element->Get<std::string>("name"));

                    ROS_INFO("Adding %s as a steering thruster", names[names.size()-1].c_str());
                }

                // register maximum effort
                if(sdf_element->HasElement("effort"))
                    max_command.push_back(sdf_element->Get<double>("effort"));
                else
                    max_command.push_back(100);
            }
            sdf_element = sdf_element->GetNextElement();
        }
    }


    void initWrenchControl(ros::NodeHandle &_control_node,
                           std::string _body_name,
                           std::vector<std::string> &_axe,
                           std::vector<std::string> &_controlled_axes)
    {
        // compute map pseudo-inverse
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

        // push control data to parameter server
        _control_node.setParam("config/body/link", _body_name);

        unsigned int thr_i, dir_i;

        // compute max force in each direction, writes controlled axes
        double thruster_max_effort;

        for(dir_i=0;dir_i<6;++dir_i)
        {
            thruster_max_effort = 0;
            for(thr_i=0;thr_i<fixed_idx.size();++thr_i)
                thruster_max_effort += max_command[fixed_idx[thr_i]] * std::abs(map(dir_i,thr_i));
            if(thruster_max_effort != 0)
            {
                _controlled_axes.push_back(_axe[dir_i]);
                char param[FILENAME_MAX];
                // push to position PID
                sprintf(param, "%s/position/i_clamp", _axe[dir_i].c_str());
                _control_node.setParam(param, thruster_max_effort);
                // push to velocity PID
                sprintf(param, "%s/velocity/i_clamp", _axe[dir_i].c_str());
                _control_node.setParam(param, thruster_max_effort);
            }
        }
    }

    void saturate(Eigen::VectorXd &_command)
    {
        double norm_ratio = 1;
        unsigned int i;
        for(i=0;i<fixed_idx.size();++i)
            norm_ratio = std::max(norm_ratio, std::abs(_command(i)) / max_command[i]);
        _command *= 1./norm_ratio;
    }

    std::vector<unsigned int> steer_idx, fixed_idx;
    std::vector<std::string> names;
    std::vector<double> max_command;

    Eigen::MatrixXd map;
    Eigen::MatrixXd inverse_map;
};

}



#endif // MAPPER_H
