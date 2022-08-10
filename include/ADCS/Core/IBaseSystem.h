#ifndef _I_BASE_SYSTEM_H
#define _I_BASE_SYSTEM_H

#include <memory> //std::shared_ptr

#include <Eigen/Core>
#include <Eigen/Geometry> // Quaternion

#include <boost/array.hpp>
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <functional> // std:: bind, std::placeholders

#ifdef BUILD_PYTHON_LIB
#include "NumPyArrayData.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace numpy = boost::python::numpy;
#endif

namespace pl = std::placeholders;
namespace odeint = boost::numeric::odeint;

template <unsigned int STATE_SIZE, unsigned int ACTION_SIZE>
class IBaseSystem {
public:
    typedef std::shared_ptr<IBaseSystem> Ptr;
    typedef boost::array<double, STATE_SIZE> state_type;
    typedef Eigen::Matrix<double, ACTION_SIZE, 1> action_type;

    IBaseSystem() { }
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

    /**
     * @brief Set the state of system
     *
     * @param state
     */
    virtual void set_state(const state_type& state) = 0;
    /**
     * @brief Get the state of system
     *
     * @param state
     */

    virtual void get_state(state_type& state) = 0;
    /**
     * @brief Eqauation of motion of system with state X in form of
     *          dX/dt = f(x,t);
     *
     * @param x_  system state
     * @param dxdt_ state derivative
     * @param t  time
     */
    virtual void operator()(const state_type& x_, state_type& dxdt_, double t) = 0;

    virtual action_type calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t = 0);
    /**
     * @brief Set control action variable, action is controlable variable in system
     *
     * @param action
     */
    void set_action(const action_type& action)
    {
        _action = action;
    }

    /**
     * @brief Time evaluation step, numerical integrate in time with control action
     *
     * @param action
     * @param t_start
     * @param t_end
     * @param dt
     */
    void step(const action_type& action = Eigen::Matrix<double, ACTION_SIZE, 1>::setZero(), const double& t_start = 0.0, const double& t_end = 0.01, const double& dt = 0.001)
    {
        // get current state
        state_type X;
        get_state(X);
        // set action
        set_action(action);
        // integrate in time
        integrate_const(stepper,
            std::bind(&IBaseSystem::operator(), std::ref(*this), pl::_1, pl::_2, pl::_3),
            X, t_start, t_end, dt);
        // update current state
        set_state(X);
    }

#ifdef BUILD_PYTHON_LIB
    // Python wrappers
    void py_set_state(const numpy::ndarray& state)
    {
        state_type X;
        NumPyArrayData<double> d(state);
        for (size_t i = 0; i < STATE_SIZE; i++) {
            X[i] = d(i);
        }
        set_state(X);
    }
    numpy::ndarray py_get_state()
    {
        state_type state;
        get_state(state);
        Py_intptr_t shape[1] = { state.size() };
        numpy::ndarray result = numpy::zeros(1, shape, numpy::dtype::get_builtin<double>());
        std::copy(state.begin(), state.end(), reinterpret_cast<double*>(result.get_data()));
        return result;
    }
    numpy::ndarray py_step(const numpy::ndarray& action, const double& t_start = 0.0, const double& t_end = 0.01, const double& dt = 0.001)
    {
        action_type u;
        NumPyArrayData<double> data(action);
        for (size_t i = 0; i < ACTION_SIZE; i++) {
            u[i] = data(i);
        }

        step(u);
        return py_get_state();
    }

    numpy::ndarray py_calc_steering(const numpy::ndarray& torque, const double& t)
    {
        NumPyArrayData<double> data(torque);
        Eigen::Vector3d u(data(0), data(1), data(2));
        action_type act = calc_steering(u, t);

        Py_intptr_t shape[1] = { ACTION_SIZE };
        numpy::ndarray result_vector = numpy::zeros(1, shape, numpy::dtype::get_builtin<double>());
        for (size_t i = 0; i < ACTION_SIZE; i++) {
            result_vector[i] = act[i];
        }
        return result_vector;
    }
#endif

protected:
    // body orientation quaternion
    Eigen::Quaternion<double> _quaternion;

    // body angular velocity
    Eigen::Matrix<double, 3, 1> _rate;

    // body inertia
    Eigen::Matrix<double, 3, 3> _Jp;

    // control action vector
    action_type _action;

    // state vector
    state_type _state;

    // numeric integrator type
    odeint::runge_kutta4<state_type> stepper;
};
#endif //#ifndef _I_BASE_SYSTEM_H