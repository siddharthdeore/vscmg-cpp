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

    void updateCMGAxes();

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
        os << "Quat  : " << obj._state[0] << ", " << obj._state[1] << ", " << obj._state[2] << ", " << obj._state[3] << "\n";
        os << "Rate  : " << obj._state[4] << ", " << obj._state[5] << ", " << obj._state[6] << "\n";
        os << "delta : " << obj._state[7] << ", " << obj._state[8] << ", " << obj._state[9] << ", " << obj._state[10] << "\n";
        os << "Omega : " << obj._state[11] << ", " << obj._state[12] << ", " << obj._state[13] << ", " << obj._state[14] << "\n";
        return os;
    }

private:
    double _beta; // skew angle of pyramid

    Eigen::Matrix<double, 4, 1> _delta; // gimbal angles
    Eigen::Matrix<double, 4, 1> _Omega; // reaction wheel velocities

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