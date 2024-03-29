#ifndef _ADCS_RW4_BODY_H
#define _ADCS_RW4_BODY_H

#pragma once

#include <ADCS/Core/IBaseSystem.h>
#include <ADCS/Core/kinematics.h>

#include <iomanip>

class RW4 : public IBaseSystem<11, 4> {
public:
    RW4();

    RW4(
        const Eigen::Quaternion<double>& q,
        const Eigen::Matrix<double, 3, 1>& w,
        const Eigen::Matrix<double, 4, 1>& Omega,
        const double& Jw);

    ~RW4();

    void operator()(const state_type& x, state_type& dxdt, double t);

    friend std::ostream& operator<<(std::ostream& os, const RW4& obj)
    {
        os << std::setw(6) << std::setprecision(4) << std::fixed;
        os << "Quat  : " << obj._state[0] << ", " << obj._state[1] << ", " << obj._state[2] << ", " << obj._state[3] << "\n";
        os << "Rate  : " << obj._state[4] << ", " << obj._state[5] << ", " << obj._state[6] << "\n";
        return os;
    }
    action_type calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t = 0);

private:
    Eigen::Matrix<double, 4, 1> _Omega; // reaction wheel velocities
    Eigen::Matrix<double, 3, 4> Gs;
    Eigen::Matrix<double, 4, 3> Gs_psudo_inv;
    double _Jw = 0.01;
};

#endif