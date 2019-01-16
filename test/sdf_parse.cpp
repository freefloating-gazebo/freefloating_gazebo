#include <ros/ros.h>
#include <freefloating_gazebo/hydro_model_parser.h>

using namespace std;

int main(int argc, char ** argv)
{
ros::init(argc, argv, "sdf_parse");
ros::NodeHandle nh;

ffg::HydroModelParser parser;
parser.parseAll(nh);
const auto link = parser.getLink("base_link");
auto map = parser.thrusterMap();
std::cout << "Map: " << map << std::endl;
std::cout << "Inertia: \n" << link.inertia << std::endl;
if(link.has_added_mass)
  std::cout << "Added mass: \n" << link.added_mass << std::endl;
if(link.has_lin_damping)
  std::cout << "Linear damping: " << link.lin_damping.transpose() << std::endl;
if(link.has_quad_damping)
  std::cout << "Quadratic damping: " << link.quad_damping.transpose() << std::endl;
}
