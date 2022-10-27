#include <ADCS/Systems/RigidBody.h>
RigidBody::RigidBody()
{
    _Jp = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
    _state = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
}
RigidBody::RigidBody(const Eigen::Quaternion<double>& q, const Eigen::Matrix<double, 3, 1>& w)
{
    _state = { q.w(), q.x(), q.y(), q.z(), w.x(), w.y(), w.z() };
    _Jp = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
}

RigidBody::~RigidBody() { }

void RigidBody::operator()(const state_type& x, state_type& dxdt, double t)
{
    const Eigen::Quaterniond q(x[0], x[1], x[2], x[3]);
    const Eigen::Vector3d w(x[4], x[5], x[6]);

    // kinematics
    const Eigen::Quaterniond q_dot = get_quaternion_kinematics(q, w);

    // rigid body dymaics Jw_dot + w^Jw = u
    const Eigen::Vector3d w_dot = -_Jp.inverse() * (w.cross(_Jp * w) - _action);

    dxdt[0] = q_dot.w();
    dxdt[1] = q_dot.x();
    dxdt[2] = q_dot.y();
    dxdt[3] = q_dot.z();

    dxdt[4] = w_dot.x();
    dxdt[5] = w_dot.y();
    dxdt[6] = w_dot.z();
}
RigidBody::action_type RigidBody::calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t)
{
    return torque;
}

#ifdef BUILD_PYTHON_LIB
EXPOSE_SYSTEM_TO_PYTHON(RigidBody);
#endif