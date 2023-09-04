#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <ros/ros.h>
using namespace std;
namespace Biped
{
    class biped_robot
    {
    private:
        double _a;
        double _b;
        double _l;
        double _m;
        double _mb;
        double _g;
        double _J;
        double _slope;
        Eigen::Vector2d _theta;
        Eigen::Vector2d _vtheta;
        bool _support_trans;
        double _dt;
        vector<Eigen::Vector4d> _traj;

    public:
        biped_robot(Eigen::Vector2d theta,
                    Eigen::Vector2d vtheta,
                    double slope = 2 * M_PI / 180,
                    double dt = 0.001,
                    double a = 0.5,
                    double b = 0.5,
                    double m = 5,
                    double mb = 0,
                    double g = 9.8,
                    double J = 0.4167);
        ~biped_robot();
        Eigen::Matrix<double, 2, 2> M(Eigen::Vector2d &) const;
        Eigen::Matrix<double, 2, 2> C(Eigen::Vector2d &, Eigen::Vector2d &) const;
        Eigen::Vector2d G(Eigen::Vector2d &) const;
        Eigen::Vector2d getTheta() const;
        Eigen::Vector2d getVtheta() const;
        Eigen::Matrix<double, 2, 2> Q(Eigen::Vector2d &) const;
        Eigen::Vector4d EulerLagrange(Eigen::Vector4d &&);
        void ssPhase();
        void dsPhase();
        bool collision() const;
        void walking();
        void showTraj() const;
    };
}
