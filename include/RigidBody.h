#ifndef _RIGID_BODY_H
#define _RIGID_BODY_H

#pragma once
#include "IBaseSystem.h"
#include "kinematics.h"
#include <iomanip>


class RigidBody : public IBaseSystem<7> {
public:
    RigidBody()
    {
        _quaternion = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
        _rate = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
        _Jp = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
    }
    RigidBody(const Eigen::Quaternion<double>& q, const Eigen::Matrix<double, 3, 1>& w)
    {
        _quaternion = q;
        _rate = w;
    }

    ~RigidBody() { }

    void set_state(const state_type& state)
    {
        _quaternion.w() = state[0];
        _quaternion.x() = state[1];
        _quaternion.y() = state[2];
        _quaternion.z() = state[3];
        _quaternion.normalize();

        _rate[0] = state[4];
        _rate[1] = state[5];
        _rate[2] = state[6];
    }
    void get_state(state_type& state)
    {
        state[0] = _quaternion.w();
        state[1] = _quaternion.x();
        state[2] = _quaternion.y();
        state[3] = _quaternion.z();

        state[4] = _rate.x();
        state[5] = _rate.y();
        state[6] = _rate.z();
    }
    void operator()(const state_type& x, state_type& dxdt, double t)
    {
        const Eigen::Quaterniond q(x[0], x[1], x[2], x[3]);
        const Eigen::Vector3d w(x[4], x[5], x[6]);

        // kinematics
        const Eigen::Quaterniond q_dot = get_quaternion_kinematics(q, w);

        // rigid body dymaics Jw = w^Jw
        const Eigen::Vector3d w_dot = -_Jp.inverse() * w.cross(_Jp * w);

        dxdt[0] = q_dot.w();
        dxdt[1] = q_dot.x();
        dxdt[2] = q_dot.y();
        dxdt[3] = q_dot.z();

        dxdt[4] = w_dot.x();
        dxdt[5] = w_dot.y();
        dxdt[6] = w_dot.z();
    }
    friend std::ostream& operator<<(std::ostream& os, const RigidBody& obj)
    {
        os << std::setw(6) << std::setprecision(4) << std::fixed;
        os << "Quat  : " << obj._quaternion.w() << ", " << obj._quaternion.x() << ", " << obj._quaternion.y() << ", " << obj._quaternion.z() << "\n";
        os << "Rate  : " << obj._rate.transpose() << "\n";
        return os;
    }

};

#endif