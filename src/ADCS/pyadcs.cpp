#include <ADCS/Core/Controllers.h>
#include <ADCS/Systems/RigidBody.h>
#include <ADCS/Systems/VSCMG.h>

#ifdef BUILD_PYTHON_LIB
numpy::ndarray py_quaternion_control(const numpy::ndarray& state, const numpy::ndarray& des_state, const double& K = 10.0, const double C = 1.0)
{
    NumPyArrayData<double> data(state);
    NumPyArrayData<double> des_data(des_state);

    const Eigen::Quaterniond q(data(0), data(1), data(2), data(3));
    const Eigen::Quaterniond qd(des_data(0), des_data(1), des_data(2), des_data(3));
    const Eigen::Vector3d w(data(4), data(5), data(6));
    const Eigen::Quaterniond qe = get_quaternion_error(q, qd);
    const Eigen::Vector3d u = Controller::quaternion_feedback(qe, w, K, C);
    Py_intptr_t shape[1] = { u.size() };
    numpy::ndarray result = numpy::zeros(1, shape, numpy::dtype::get_builtin<double>());
    result[0] = u[0];
    result[1] = u[1];
    result[2] = u[2];
    return result;
}

BOOST_PYTHON_MODULE(pyadcs)
{
    using namespace boost::python;
    numpy::initialize();
    Py_Initialize();

    def("quaternion_control", &py_quaternion_control);

    class_<RigidBody>("RigidBody")
        .def("get_state", &RigidBody::py_get_state, "Returns numpy array of Rigid Body State vector")
        .def("set_state", &RigidBody::py_set_state, "Set Rigid Body state vector with numpy array")
        .def("step", &RigidBody::py_step, "Evaluation of system dynamics in time")
        .def("calc_steering", &RigidBody::py_calc_steering, "Transformation of torque vector to control action");

    class_<VSCMG>("VSCMG")
        .def("get_state", &VSCMG::py_get_state, "Returns numpy array of VSCMG State vector")
        .def("set_state", &VSCMG::py_set_state, "Set VSCMG state vector with numpy array")
        .def("step", &VSCMG::py_step, "Evaluation of system dynamics in time")
        .def("calc_steering", &VSCMG::py_calc_steering, "Transformation of torque vector to control action");
}
#endif