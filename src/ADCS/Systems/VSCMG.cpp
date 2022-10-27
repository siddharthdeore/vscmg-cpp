#include <ADCS/Systems/VSCMG.h>
#include <Eigen/SVD>
#include <math.h> /* sin, cos */

VSCMG::VSCMG()
{
    _beta = 0.9553166181245093,
    _Jp = Eigen::Matrix<double, 3, 3>::Identity(3, 3),
    _Jw = 0.01,
    _Jcmg = 0.01,
    _Jt = 0.01;

    _state = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
}
VSCMG::VSCMG(const double& beta,
    const Eigen::Matrix<double, 3, 3>& Jp,
    const double& Jw,
    const double& Jcmg,
    const double& Jt)
    : _beta(beta)
    , _Jp(Jp)
    , _Jcmg(Jcmg)
    , _Jt(Jt)
{
    _state = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
}
VSCMG::~VSCMG()
{
}

void VSCMG::set_gimbal_angle(const Eigen::Matrix<double, 4, 1>& delta)
{
    _delta = delta;
    updateCMGAxes();
}
void VSCMG::set_wheel_velocity(const Eigen::Matrix<double, 4, 1>& Omega)
{
    _Omega = Omega;
}

void VSCMG::updateCMGAxes()
{
    const auto sb = std::sin(_beta);
    const auto cb = std::cos(_beta);
    const auto cd = _delta.array().cos();
    const auto sd = _delta.array().sin();

    Gg << -sb, 0, sb, 0,
        0, sb, 0, -sb,
        cb, cb, cb, cb;

    Gs << cb * cd(0), -sd(1), -cb * cd(2), sd(3),
        -sd(0), -cb * cd(1), sd(2), cb * cd(3),
        sb * cd(0), sb * cd(1), sb * cd(2), sb * cd(3);

    Gt << cb * sd(0), cd(1), -cb * sd(2), -cd(3),
        cd(0), -cb * sd(1), -cd(2), cb * sd(3),
        sb * sd(0), sb * sd(1), sb * sd(2), sb * sd(3);
}

Eigen::Matrix<double, 3, 4> VSCMG::get_gimbal_matrix() const
{
    return Gg;
}

Eigen::Matrix<double, 3, 4> VSCMG::get_spin_matrix() const
{
    return Gs;
}

Eigen::Matrix<double, 3, 4> VSCMG::get_transverse_matrix() const
{
    return Gt;
}

Eigen::Matrix<double, 8, 1> VSCMG::calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t)
{

    const auto D = _Jw * Gs; // RW
    const auto C = _Jw * (Gt * _Omega.asDiagonal()); // CMG
    Q << C, D;

    const auto cm = (Gt * Gt.transpose()).determinant(); // CMG Singularity coeficient
    const auto wm = (Gs * Gs.transpose()).determinant(); // RW Singularity coeficient
    const auto vm = (Q * Q.transpose()).determinant(); // RW Singularity coeficient

    // Singularity avoiding Steering Law
    const auto beta = std::min(10000.0 * std::exp(-0.0001 * cm / (wm + 1e-6)), 10000.0);

    // Weight matrix
    Eigen::Matrix<double, 8, 1> vec;
    vec << 1.0, 1.0, 1.0, 1.0, beta, beta, beta, beta;
    const auto W = vec.asDiagonal();

    // Avoid singularities using SVD
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 8>> svd(Q, Eigen::ComputeFullV);
    const auto gamma = 0.0001 * std::exp(-10 * vm);
    Eigen::Matrix<double, 8, 1> V = gamma * svd.matrixV().col(0);

    // Escape Singularities using off diagonal harmonics
    const auto alpha = 0.001 * std::exp(-10 * cm);

    const auto e1 = alpha * std::sin(0.01 * t);
    const auto e2 = alpha * std::sin(0.01 * t + M_PI_2);
    const auto e3 = alpha * std::sin(0.01 * t + M_PI);

    // Off diagonal harmonics
    Eigen::Matrix<double, 3, 3> E;
    E << 1.0, e3, e2,
        e3, 1.0, e1,
        e2, e1, 1.0;

    const auto QWQT = Q * W * Q.transpose();

    // Singularity Escaping Steering step
    const Eigen::Matrix<double, 8, 1> delta_Omega_dot = W * Q.transpose() * (QWQT + E).inverse() * torque + V;
    return delta_Omega_dot;
}

void VSCMG::operator()(const state_type& x, state_type& dxdt, double t)
{
    const Eigen::Quaterniond q(x[0], x[1], x[2], x[3]);
    const Eigen::Vector3d w(x[4], x[5], x[6]);
    const Eigen::Matrix<double, 4, 1> delta(x[7], x[8], x[9], x[10]);
    const Eigen::Matrix<double, 4, 1> Omega(x[11], x[12], x[13], x[14]);
    // CMG direction coisins
    set_gimbal_angle(delta);
    set_wheel_velocity(Omega);

    const Eigen::Matrix<double, 3, 4> Gs = get_spin_matrix();
    const Eigen::Matrix<double, 3, 4> Gt = get_transverse_matrix();
    const Eigen::Matrix<double, 3, 4> Gg = get_gimbal_matrix();
    // System Inertia Tensor
    const Eigen::Matrix<double, 3, 3> J = _Jp + _Jw * Gs * Gs.transpose() + _Jt * Gg * Gg.transpose() + _Jcmg * Gt * Gt.transpose();

    // kinematics
    const Eigen::Quaterniond q_dot = get_quaternion_kinematics(q, w);

    const Eigen::Matrix<double, 8, 1> delta_Omega_dot = -_action;

    // gimbal velocity deviation
    const auto delta_dot = delta_Omega_dot.block<4, 1>(0, 0);
    const auto Omega_dot = delta_Omega_dot.block<4, 1>(4, 0);

    /* dynamics */

    // Gimbal velocity contribution
    const Eigen::Matrix<double, 3, 1> term1 = (Gt * (delta_dot.asDiagonal() * (_Jw - _Jt)) * Gs.transpose() + Gs * ((delta_dot.asDiagonal()) * (_Jw - _Jt)) * Gt.transpose()) * w;

    // Reaction wheel contribution
    const Eigen::Matrix<double, 3, 1> term2 = Gt * (_Jw * Omega.asDiagonal() * delta_dot) + Gs * (_Jw * Omega_dot);

    // Gyroscopic effects due to variation of angular momentum of entire system
    const Eigen::Matrix<double, 3, 1> term3 = w.cross(J * w + Gg * _Jcmg * delta_dot + Gs * _Jw * Omega);

    // eqation of motion of VSCMG dynamics
    const Eigen::Matrix<double, 3, 1> w_dot = J.inverse() * (-term1 - term2 - term3);

    dxdt[0] = q_dot.w();
    dxdt[1] = q_dot.x();
    dxdt[2] = q_dot.y();
    dxdt[3] = q_dot.z();

    dxdt[4] = w_dot.x();
    dxdt[5] = w_dot.y();
    dxdt[6] = w_dot.z();

    dxdt[7] = delta_dot[0];
    dxdt[8] = delta_dot[1];
    dxdt[9] = delta_dot[2];
    dxdt[10] = delta_dot[3];

    dxdt[11] = Omega_dot[0];
    dxdt[12] = Omega_dot[1];
    dxdt[13] = Omega_dot[2];
    dxdt[14] = Omega_dot[3];
}

#ifdef BUILD_PYTHON_LIB
EXPOSE_SYSTEM_TO_PYTHON(VSCMG)
#endif