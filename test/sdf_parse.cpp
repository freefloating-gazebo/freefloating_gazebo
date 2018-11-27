#include <ros/ros.h>
#include <freefloating_gazebo/hydro_model_parser.h>

using namespace std;

int main(int argc, char ** argv)
{
ros::init(argc, argv, "sdf_parse");
ros::NodeHandle nh;

ffg::HydroModelParser parser;
parser.parseAll(nh);
auto map = parser.thrusterMap();
//std::cout << "Map: " << map << std::endl;

}
