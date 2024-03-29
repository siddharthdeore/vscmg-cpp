#ifndef _ADCS_RIGID_BODY_H
#define _ADCS_RIGID_BODY_H

#pragma once

#include <ADCS/Core/IBaseSystem.h>
#include <ADCS/Core/kinematics.h>

#include <iomanip>

class RigidBody : public IBaseSystem<7, 3> {
public:
    RigidBody();

    RigidBody(
        const Eigen::Quaternion<double>& q,
        const Eigen::Matrix<double, 3, 1>& w);

    ~RigidBody();

    void operator()(const state_type& x, state_type& dxdt, double t);

    friend std::ostream& operator<<(std::ostream& os, const RigidBody& obj)
    {
        os << std::setw(6) << std::setprecision(4) << std::fixed;
        os << "Quat  : " << obj._state[0] << ", " << obj._state[1] << ", " << obj._state[2] << ", " << obj._state[3] << "\n";
        os << "Rate  : " << obj._state[4] << ", " << obj._state[5] << ", " << obj._state[6] << "\n";
        return os;
    }
    action_type calc_steering(const Eigen::Matrix<double, 3, 1>& torque, const double& t = 0);
};

#endif