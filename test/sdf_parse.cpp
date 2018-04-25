#include <sdf/sdf.hh>
#include <sdf/parser.hh>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
ros::init(argc, argv, "butterWorth");
ros::NodeHandle nh;

std::string s;
nh.getParam("/auv/robot_description", s);

sdf::SDFPtr sdfElement(new sdf::SDF());
sdf::init(sdfElement);
sdf::initString(s, sdfElement);

sdf::SDFPtr root;
sdf::initString(s, root);

sdf::ElementPtr elem;


}
