#include "pdw_sim/biped_robot.h"
using namespace Biped;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "biped_walking_node");
    std::vector<double> initState;
    double a,b,m,mb,g,J,slope;
    ros::param::get("a", a);
    ros::param::get("b", b);
    ros::param::get("m", m);
    ros::param::get("mb", mb);
    ros::param::get("g", g);
    ros::param::get("J", J);
    ros::param::get("slope", slope);
    ros::param::get("init_state", initState); 
    Eigen::Vector2d theta(initState[0], initState[1]);
    Eigen::Vector2d vtheta(initState[2], initState[3]);
    cout << theta << vtheta << endl;
    // biped_robot br(theta, vtheta);
    // ROS_INFO_STREAM("a" << a << "b" << b << "slope" << slope);
    biped_robot br(theta, vtheta, slope* M_PI / 180, 0.001,a,b,m,mb,g,J);
    br.walking();
    return 0;
}
