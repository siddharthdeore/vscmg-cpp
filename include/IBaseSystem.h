#ifndef _I_BASE_SYSTEM_H
#define _I_BASE_SYSTEM_H

#include <memory> //std::shared_ptr

#include <Eigen/Core>
#include <Eigen/Geometry> // Quaternion

#include <boost/array.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <functional> // std:: bind, std::placeholders

namespace pl = std::placeholders;
namespace odeint = boost::numeric::odeint;

template <unsigned int STATE_SIZE>
class IBaseSystem {
public:
    typedef std::shared_ptr<IBaseSystem> Ptr;
    typedef boost::array<double, STATE_SIZE> state_type;

    IBaseSystem()
    {
    }
    ~IBaseSystem() { }
    void set_inertia(const Eigen::Matrix<double, 3, 3>& Jp)
    {
        _Jp = Jp;
    }
    void set_inertia(const Eigen::Matrix<double, 3, 1>& JpVec)
    {
        _Jp = Eigen::Vector3d(JpVec).asDiagonal();
    }
    void set_inertia(const double& Ixx, const double& Iyy, const double& Izz)
    {
        _Jp = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();
    }

    virtual void set_state(const state_type& state) = 0;
    virtual void get_state(state_type& state) = 0;
    virtual void operator()(const state_type& x_, state_type& dxdt_, double t) = 0;
    virtual void step(const double& t_start, const double& t_end, const double& dt) = 0;

protected:
    Eigen::Quaternion<double> _quaternion;
    Eigen::Matrix<double, 3, 1> _rate;
    Eigen::Matrix<double, 3, 3> _Jp;
    odeint::runge_kutta4<state_type> stepper;
};
#endif //#ifndef _I_BASE_SYSTEM_H