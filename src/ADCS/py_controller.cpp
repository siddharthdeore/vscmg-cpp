#ifdef BUILD_PYTHON_LIB
#include <ADCS/Core/Controllers.h>
#include <ADCS/Core/NumPyArrayData.h>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace numpy = boost::python::numpy;

numpy::ndarray py_quaternion_control(
    const numpy::ndarray& state,
    const numpy::ndarray& des_state,
    const double& K = 10.0, const double C = 1.0)
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

BOOST_PYTHON_MODULE(libController)
{
    using namespace boost::python;
    numpy::initialize();
    Py_Initialize();

    def("quaternion_control", &py_quaternion_control);
}
#endif