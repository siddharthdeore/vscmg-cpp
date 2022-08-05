#include <VSCMG.h>
#include <math.h> /* sin, cos */
#include <Eigen/SVD>
VSCMG::~VSCMG()
{
}

void VSCMG::updateCMGAxes()
{
    /*
    const auto sb = std::sin(_beta);
    const auto cb = std::cos(_beta);
    const auto cd = _delta.cos();
    const auto sd = _delta.sin();

    Gg << -sb, 0, sb, 0,
        0, sb, 0, -sb,
        cb, cb, cb, cb;

    Gs << cb * cd(0), -sd(1), -cb * cd(2), sd(3),
        -sd(0), -cb * cd(1), sd(2), cb * cd(3),
        sb * cd(0), sb * cd(1), sb * cd(2), sb * cd(3);

    Gt << cb * sd(0), cd(1), -cb * sd(2), -cd(3),
        cd(0), -cb * sd(1), -cd(2), cb * sd(3),
        sb * sd(0), sb * sd(1), sb * sd(2), sb * sd(3);
        */
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

Eigen::Matrix<double, 8, 1> VSCMG::calc_steering(const Eigen::Matrix<double, 3, 1>& torque)
{
    updateCMGAxes();

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
    Eigen::JacobiSVD<Eigen::Matrix<double,3,8>> svd(Q, Eigen::ComputeThinV);

    const auto gamma = 0.0001 * std::exp(-10 * vm);
}