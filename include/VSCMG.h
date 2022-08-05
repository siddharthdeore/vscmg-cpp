#ifndef VSCMG_H
#define VSCMG_H

#pragma once
#include <kinematics.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

typedef Eigen::Matrix<double, 4, 1> RWVelocities;
typedef Eigen::Matrix<double, 4, 1> GimbalAngles;

class VSCMG {
public:
    typedef std::shared_ptr<VSCMG> Ptr;

    VSCMG(const double& beta,
        const Eigen::Matrix<double, 3, 3>& Jp,
        const double& Jw,
        const double& Jcmg,
        const double& Jt);

    ~VSCMG();

    void updateCMGAxes();
    Eigen::Matrix<double, 3, 4> get_gimbal_matrix() const;
    Eigen::Matrix<double, 3, 4> get_spin_matrix() const;
    Eigen::Matrix<double, 3, 4> get_transverse_matrix() const;

    Eigen::Matrix<double, 3, 1> calc_torque(
        const Eigen::Quaternion<double>& qe,
        const Eigen::Matrix<double, 3, 1>& we,
        const double& Kq,
        const double& Kw);

    Eigen::Matrix<double, 8, 1> calc_steering(const Eigen::Matrix<double, 3, 1>& torque);

private:
    double _beta;

    Eigen::Quaternion<double> _quaternion;
    Eigen::Matrix<double, 3, 1> _rate;

    Eigen::Matrix<double, 4, 1> _delta; // gimbal angles
    Eigen::Matrix<double, 4, 1> _Omega; // reaction wheel velocities

    Eigen::Matrix<double, 4, 1> _delta_dot; // gimbal velocities
    Eigen::Matrix<double, 4, 1> _Omega_dot; // reaction wheel accelerations

    Eigen::Matrix<double, 3, 4> Gg; // Gimbal Axis Matrix
    Eigen::Matrix<double, 3, 4> Gs; // Spin Axis Matrix
    Eigen::Matrix<double, 3, 4> Gt; // Transverse Axis Matrix
    Eigen::Matrix<double, 3, 8> Q; // [Gt Gs]

    Eigen::Matrix<double, 3, 3> _Jp; // platform inertia

    double _Jw; // wheel inertia about spin axis
    double _Jt; // cmg transvers
    double _Jcmg; // cmg inertia about gimbal axis
};

#endif