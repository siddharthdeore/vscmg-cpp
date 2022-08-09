#ifndef _ADCS_RIGID_BODY_H
#define _ADCS_RIGID_BODY_H

#pragma once

#include <ADCS/Core/kinematics.h>
#include <ADCS/Core/IBaseSystem.h>

#include <iomanip>

class RigidBody : public IBaseSystem<7, 3> {
public:
    RigidBody();

    RigidBody(
        const Eigen::Quaternion<double>& q,
        const Eigen::Matrix<double, 3, 1>& w);

    ~RigidBody();

    void set_state(const state_type& state);

    void get_state(state_type& state);

    void operator()(const state_type& x, state_type& dxdt, double t);

    friend std::ostream& operator<<(std::ostream& os, const RigidBody& obj)
    {
        os << std::setw(6) << std::setprecision(4) << std::fixed;
        os << "Quat  : " << obj._quaternion.w() << ", " << obj._quaternion.x() << ", " << obj._quaternion.y() << ", " << obj._quaternion.z() << "\n";
        os << "Rate  : " << obj._rate.transpose() << "\n";
        return os;
    }
};

#endif