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

#define EXPOSE_SYSTEM_TO_PYTHON(PYSYS_NAME)                                                                           \
    BOOST_PYTHON_MODULE(lib##PYSYS_NAME)                                                                              \
    {                                                                                                                 \
        using namespace boost::python;                                                                                \
        numpy::initialize();                                                                                          \
        Py_Initialize();                                                                                              \
        class_<PYSYS_NAME>(#PYSYS_NAME)                                                                               \
            .def("get_state", &PYSYS_NAME::py_get_state, "Returns numpy array of #PYSYS_NAME State vector")           \
            .def("set_state", &PYSYS_NAME::py_set_state, "Set #PYSYS_NAME state vector with numpy array")             \
            .def("step", &PYSYS_NAME::py_step, "Evaluation of system dynamics in time")                               \
            .def("calc_steering", &PYSYS_NAME::py_calc_steering, "Transformation of torque vector to control action") \
            .def("get_sample_action", &PYSYS_NAME::py_get_sample_action, "Get random action type array ");            \
    }
#endif

namespace pl = std::placeholders;
namespace odeint = boost::numeric::odeint;
template <std::size_t STATE_SIZE = 7, std::size_t ACTION_SIZE = 3>
class IBaseSystem {
public:
    typedef std::shared_ptr<IBaseSystem> Ptr;
    typedef boost::array<double, STATE_SIZE> state_type;
    typedef Eigen::Matrix<double, ACTION_SIZE, 1> action_type;

    IBaseSystem() { }
    ~IBaseSystem() { }

    inline void set_inertia(const Eigen::Matrix<double, 3, 3>& Jp)
    {
        _Jp = Jp;
    }

    inline void set_inertia(const Eigen::Matrix<double, 3, 1>& JpVec)
    {
        _Jp = Eigen::Vector3d(JpVec).asDiagonal();
    }

    inline void set_inertia(const double& Ixx, const double& Iyy, const double& Izz)
    {
        _Jp = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();
    }

    /**
     * @brief Set the state of system
     *
     * @param state
     */
    inline void set_state(const state_type& state)
    {
        _state = std::move(state);
    }

    /**
     * @brief Get the state of system
     *
     * @param state
     */
    inline state_type get_state() const
    {
        return _state;
    }

    /**
     * @brief Eqauation of motion of system with state X in form of
     *          dX/dt = f(x,t);
     *
     * @param x_  system state
     * @param dxdt_ state derivative
     * @param t  time
     */
    virtual void operator()(const state_type& x_, state_type& dxdt_, double t) = 0;

    /**
     * @brief transformation of torque in body frame to controlable actuator signal vector
     *
     * @param torque
     * @param t
     * @return action_type
     */
    virtual action_type calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t = 0) = 0;

    /**
     * @brief Set control action variable, action is controlable variable in system
     *
     * @param action
     */
    inline void set_action(const action_type& action)
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
    inline void step(const action_type& action = Eigen::Matrix<double, ACTION_SIZE, 1>::setZero(), const double& t_start = 0.0, const double& t_end = 0.01, const double& dt = 0.001)
    {
        // set action
        set_action(action);
        // integrate in time
        integrate_const(stepper,
            std::bind(&IBaseSystem::operator(), std::ref(*this), pl::_1, pl::_2, pl::_3),
            _state, t_start, t_end, dt);
    }

    inline action_type get_sample_action()
    {
        action_type x;
        x.setRandom();
        return x;
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
        Py_intptr_t shape[1] = { _state.size() };
        numpy::ndarray result = numpy::zeros(1, shape, numpy::dtype::get_builtin<double>());
        std::copy(_state.begin(), _state.end(), reinterpret_cast<double*>(result.get_data()));
        return result;
    }

    numpy::ndarray py_get_sample_action()
    {
        action_type act = get_sample_action();

        // Eigen3::Matrix to numpy array
        Py_intptr_t shape[1] = { ACTION_SIZE };
        numpy::ndarray result_vector = numpy::zeros(1, shape, numpy::dtype::get_builtin<double>());
        for (size_t i = 0; i < ACTION_SIZE; i++) {
            result_vector[i] = act[i];
        }
        return result_vector;
    }

    numpy::ndarray py_step(const numpy::ndarray& action, const double& t_start = 0.0, const double& t_end = 0.01, const double& dt = 0.001)
    {
        action_type u;

        // numpy array to Eigen3::Matrix
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

        // Eigen3::Matrix to numpy array
        Py_intptr_t shape[1] = { ACTION_SIZE };
        numpy::ndarray result_vector = numpy::zeros(1, shape, numpy::dtype::get_builtin<double>());
        for (size_t i = 0; i < ACTION_SIZE; i++) {
            result_vector[i] = act[i];
        }
        return result_vector;
    }
#endif

protected:
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