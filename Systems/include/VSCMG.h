#ifndef VSCMG_H
#define VSCMG_H

#pragma once

#include <ADCS/Core/IBaseSystem.h>
#include <ADCS/Core/kinematics.h>

#include <iomanip>
#include <memory>

class VSCMG : public IBaseSystem<15, 8> {
public:
    VSCMG();

    VSCMG(const double& beta,
        const Eigen::Matrix<double, 3, 3>& Jp,
        const double& Jw,
        const double& Jcmg,
        const double& Jt);

    ~VSCMG();

    void operator()(const state_type& x_, state_type& dxdt_, double t);

    void set_state(const state_type& x_);

    void get_state(state_type& x_);

    void updateCMGAxes();

    void set_state(
        const Eigen::Quaterniond q,
        const Eigen::Vector3d w,
        const Eigen::Matrix<double, 4, 1>& delta,
        const Eigen::Matrix<double, 4, 1>& Omega);

    void set_gimbal_angle(const Eigen::Matrix<double, 4, 1>& delta);

    void set_wheel_velocity(const Eigen::Matrix<double, 4, 1>& Omega);

    /**
     * @brief Matrix of each column represents orientation of gimbal axis
     *
     * @return Eigen::Matrix<double, 3, 4>
     */
    Eigen::Matrix<double, 3, 4> get_gimbal_matrix() const;

    /**
     * @brief Matrix of each column represents orientation of Reaction wheel spin axis
     *
     * @return Eigen::Matrix<double, 3, 4>
     */
    Eigen::Matrix<double, 3, 4> get_spin_matrix() const;

    /**
     * @brief Matrix of each column represents orientation of cross product of
     * Reaction wheel spin axis and gimbal axis.
     *
     * @return Eigen::Matrix<double, 3, 4>
     */
    Eigen::Matrix<double, 3, 4> get_transverse_matrix() const;

    action_type calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t);

    friend std::ostream& operator<<(std::ostream& os, const VSCMG& obj)
    {
        os << std::setw(6) << std::setprecision(4) << std::fixed;
        os << "Quat  : " << obj._quaternion.w() << ", " << obj._quaternion.x() << ", " << obj._quaternion.y() << ", " << obj._quaternion.z() << "\n";
        os << "Rate  : " << obj._rate.transpose() << "\n";
        os << "delta : " << obj._delta.transpose() << "\n";
        os << "Omega : " << obj._Omega.transpose() << "\n";
        return os;
    }

private:
    double _beta; // skew angle of pyramid

    Eigen::Quaternion<double> _quaternion_desired;

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