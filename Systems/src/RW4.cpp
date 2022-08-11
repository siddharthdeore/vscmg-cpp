#include "RW4.h"
RW4::RW4()
{
    _quaternion = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
    _rate = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
    _Omega = Eigen::Matrix<double, 4, 1>(0.0, 0.0, 0.0, 0.0);
    _Jp = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
    _Jw = 0.001;

    // 3 reaction wheels aligned to body axis, forth placed equidistant to all axis
    Gs << 1.0, 0, 0, -1.0 / sqrt(3.0),
        0, 1.0, 0, -1.0 / sqrt(3.0),
        0, 0, 1.0, -1.0 / sqrt(3.0);

    Gs_psudo_inv = Gs.transpose() * (Gs * Gs.transpose()).inverse();
}

RW4::RW4(
    const Eigen::Quaternion<double>& q,
    const Eigen::Matrix<double, 3, 1>& w,
    const Eigen::Matrix<double, 4, 1>& Omega,
    const double& Jw)
{
    _quaternion = q;
    _rate = w;
    _Omega = Omega;
    _Jw = Jw;

    // 3 reaction wheels aligned to body axis, forth placed equidistant to all axis
    Gs << 1.0, 0, 0, -1.0 / sqrt(3.0),
        0, 1.0, 0, -1.0 / sqrt(3.0),
        0, 0, 1.0, -1.0 / sqrt(3.0);

    Gs_psudo_inv = Gs.transpose() * (Gs * Gs.transpose()).inverse();
}

RW4::~RW4() { }

void RW4::set_state(const state_type& state)
{
    _state = state;
    _quaternion.w() = state[0];
    _quaternion.x() = state[1];
    _quaternion.y() = state[2];
    _quaternion.z() = state[3];
    _quaternion.normalize();

    _rate[0] = state[4];
    _rate[1] = state[5];
    _rate[2] = state[6];

    _Omega[0] = state[7];
    _Omega[1] = state[8];
    _Omega[2] = state[9];
    _Omega[3] = state[10];
}
void RW4::get_state(state_type& state)
{
    state[0] = _quaternion.w();
    state[1] = _quaternion.x();
    state[2] = _quaternion.y();
    state[3] = _quaternion.z();

    state[4] = _rate.x();
    state[5] = _rate.y();
    state[6] = _rate.z();

    state[7] = _Omega[0];
    state[8] = _Omega[1];
    state[9] = _Omega[2];
    state[10] = _Omega[3];
}
void RW4::operator()(const state_type& x, state_type& dxdt, double t)
{
    const Eigen::Quaterniond q(x[0], x[1], x[2], x[3]);
    const Eigen::Vector3d w(x[4], x[5], x[6]);
    const Eigen::Matrix<double, 4, 1> Omega(x[7], x[8], x[9], x[10]);

    // kinematics
    const Eigen::Quaterniond q_dot = get_quaternion_kinematics(q, w);
    action_type Omega_dot = -_action;

    // rigid body dymaics Jw_dot + w^Jw = u
    const Eigen::Vector3d rw_term = Gs * (_Jw * Omega_dot);
    const Eigen::Vector3d gyro_effect = w.cross(_Jp * w + Gs * _Jw * Omega);

    const Eigen::Vector3d w_dot = -_Jp.inverse() * (rw_term + gyro_effect);

    dxdt[0] = q_dot.w();
    dxdt[1] = q_dot.x();
    dxdt[2] = q_dot.y();
    dxdt[3] = q_dot.z();

    dxdt[4] = w_dot.x();
    dxdt[5] = w_dot.y();
    dxdt[6] = w_dot.z();

    dxdt[7] = Omega_dot[0];
    dxdt[8] = Omega_dot[1];
    dxdt[9] = Omega_dot[2];
    dxdt[10] = Omega_dot[3];
}
RW4::action_type RW4::calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t)
{
    return Gs_psudo_inv * torque;
}

#ifdef BUILD_PYTHON_LIB
EXPOSE_SYSTEM_TO_PYTHON(RW4);
#endif