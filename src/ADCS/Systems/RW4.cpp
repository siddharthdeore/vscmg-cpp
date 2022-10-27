#include  <ADCS/Systems/RW4.h>
RW4::RW4()
{
    
    _state = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0 };
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
    _state = { q.w(), q.x(), q.y(), q.z(), w.x(), w.y(), w.z(), Omega[0], Omega[1], Omega[2], Omega[3] };

    _Jw = Jw;

    // 3 reaction wheels aligned to body axis, forth placed equidistant to all axis
    Gs << 1.0, 0, 0, -1.0 / sqrt(3.0),
        0, 1.0, 0, -1.0 / sqrt(3.0),
        0, 0, 1.0, -1.0 / sqrt(3.0);

    Gs_psudo_inv = Gs.transpose() * (Gs * Gs.transpose()).inverse();
}

RW4::~RW4() { }

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