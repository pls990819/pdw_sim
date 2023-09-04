#include "pdw_sim/biped_robot.h"
#include <vector>
#include <std_msgs/Float32MultiArray.h>
using namespace std;
namespace Biped
{
    biped_robot::biped_robot(Eigen::Vector2d theta, Eigen::Vector2d vtheta, double slope, double dt, double a, double b, double m, double mb, double g, double J)
    {
        this->_a = a;
        this->_b = b;
        this->_l = a + b;
        this->_m = m;
        this->_mb = mb;
        this->_g = g;
        this->_J = J;
        this->_slope = slope;
        this->_dt = dt;
        this->_theta = theta;
        this->_vtheta = vtheta;
        this->_support_trans = false;
    }

    biped_robot::~biped_robot()
    {
    }

    Eigen::Matrix<double, 2, 2> biped_robot::M(Eigen::Vector2d &theta) const
    {
        Eigen::Matrix<double, 2, 2> _M;
        _M(0, 0) = _J + _m * pow(_a, 2) + _m * pow(_l, 2) + _mb * pow(_l, 2);
        _M(0, 1) = -_m * _l * _b * cos(theta(0) - theta(1));
        _M(1, 0) = _M(0, 1);
        _M(1, 1) = _J + _m * pow(_b, 2);
        return _M;
    }

    Eigen::Matrix<double, 2, 2> biped_robot::C(Eigen::Vector2d &theta, Eigen::Vector2d &vtheta) const
    {
        Eigen::Matrix<double, 2, 2> _C;
        _C(0, 0) = 0;
        _C(0, 1) = -_m * _l * _b * sin(theta(0) - theta(1)) * vtheta(1);
        _C(1, 0) = _m * _l * _b * sin(theta(0) - theta(1)) * vtheta(0);
        _C(1, 1) = 0;
        return _C;
    }

    Eigen::Vector2d biped_robot::G(Eigen::Vector2d &theta) const
    {
        Eigen::Vector2d _G;
        _G(0) = -(_m * _a + _mb * _l + _m * _l) * _g * sin(_slope + theta(0));
        _G(1) = _m * _g * _b * sin(_slope + theta(1));
        return _G;
    }

    Eigen::Matrix<double, 2, 2> biped_robot::Q(Eigen::Vector2d &theta) const
    {
        Eigen::Matrix<double, 2, 2> Q1;
        Eigen::Matrix<double, 2, 2> Q2;
        double alpha;
        alpha = theta(0) - theta(1);
        Q1(0, 0) = _J + _m * pow(_a, 2) + _m * pow(_l, 2) + _mb * pow(_l, 2) - _m * _l * _b * cos(alpha);
        Q1(0, 1) = _J + _m * pow(_b, 2) - _m * _l * _b * cos(alpha);
        Q1(1, 0) = -_m * _l * _b * cos(alpha);
        Q1(1, 1) = _J + _m * pow(_b, 2);

        Q2(0, 0) = _J + (2 * _m * _a * _l + _mb * pow(_l, 2)) * cos(alpha) - _m * _a * _b;
        Q2(0, 1) = _J - _m * _a * _b;
        Q2(1, 0) = Q2(0, 1);
        Q2(1, 1) = 0;
        return Q1.inverse() * Q2;
    }

    Eigen::Vector2d biped_robot::getTheta() const
    {
        return _theta;
    }

    Eigen::Vector2d biped_robot::getVtheta() const
    {
        return _vtheta;
    }

    Eigen::Vector4d biped_robot::EulerLagrange(Eigen::Vector4d &&br_state)
    {
        Eigen::Vector2d theta, vtheta, vtheta_dot;
        Eigen::Vector4d state_dot;
        theta = br_state.segment(0, 2);
        vtheta = br_state.segment(2, 2);
        vtheta_dot = -M(theta).inverse() * (C(theta, vtheta) * vtheta + G(theta));
        state_dot << vtheta, vtheta_dot;
        return state_dot;
    }
    void biped_robot::ssPhase()
    {
        Eigen::Vector4d state, newstate;
        state << _theta, _vtheta;
        Eigen::Vector4d k1, k2, k3, k4;
        k1 = _dt * EulerLagrange(move(state));
        k2 = _dt * EulerLagrange(state + 0.5 * k1);
        k3 = _dt * EulerLagrange(state + 0.5 * k2);
        k4 = _dt * EulerLagrange(state + k3);
        newstate = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
        // newstate = LungeKutta(&biped_robot::EulerLagrange,state,_dt);
        _theta = newstate.segment(0, 2);
        _vtheta = newstate.segment(2, 2);
        // cout << collision() << endl;
        _support_trans = collision();
    }
    void biped_robot::dsPhase()
    {
        Eigen::Matrix<double, 2, 2> T;
        Eigen::Vector2d newtheta, newvtheta;
        T << 0, 1, 1, 0;
        newvtheta = Q(_theta) * _vtheta;
        newtheta = T * _theta;
        _theta = newtheta;
        _vtheta = newvtheta;
        _support_trans = false;
        // cout << "ds" << _theta << _vtheta << endl;
    }

    bool biped_robot::collision() const
    {
        double X = _l*sin(_theta(0))-_l*sin(_theta(1));
        double Y = _l*cos(_theta(0))-_l*cos(_theta(1));
        double Y_dot = -_vtheta(0)*_l*sin(_theta(0))+_vtheta(1)*_l*sin(_theta(1));
        if(X > 0 && Y <= 0 && Y_dot <= 0) return true;
        return false;
    } 


    void biped_robot::walking()
    {
        ros::NodeHandle n;
        ros::Publisher bipedStatePub = n.advertise<std_msgs::Float32MultiArray>("biped_state", 10);
        ros::Rate loop_rate(1000);
        while (ros::ok())
        {   
            if (!_support_trans)
            {
                ssPhase();
                if (_support_trans)
                {
                    dsPhase();
                }
            }
            std_msgs::Float32MultiArray bipedState;
            bipedState.data.push_back(_theta(0));
            bipedState.data.push_back(_theta(1));
            bipedState.data.push_back(_vtheta(0));
            bipedState.data.push_back(_vtheta(1));
            bipedStatePub.publish(bipedState);
            loop_rate.sleep();
            ros::spinOnce();

        }
    }
    void biped_robot::showTraj() const
    {
        for (int i = 0; i < _traj.size(); i++)
        {
            cout << _traj[i] << endl;
        }
    }
}
