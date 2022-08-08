#ifndef KINEMATICS_H
#define KINEMATICS_H

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

static Eigen::Matrix<double, 3, 3> hat(
    const Eigen::Matrix<double, 3, 1>& v);

static Eigen::Quaternion<double> get_quaternion_error(
    const Eigen::Quaternion<double>& q,
    const Eigen::Quaternion<double>& qd);

static Eigen::Quaternion<double> get_quaternion_kinematics(
    const Eigen::Quaternion<double>& q,
    const Eigen::Matrix<double, 3, 1>& w);

static Eigen::Matrix<double, 3, 1> controller(
    const Eigen::Quaternion<double>& qe,
    const Eigen::Matrix<double, 3, 1>& we,
    const double& Kq,
    const double& Kw);

static Eigen::Matrix<double, 3, 1> controller(
    const Eigen::Quaternion<double>& qe,
    const Eigen::Matrix<double, 3, 1>& we,
    const double& Kq,
    const double& Kw)
{
    return Kq * qe.vec() + Kw * we;
}

static Eigen::Quaternion<double> get_quaternion_kinematics(
    const Eigen::Quaternion<double>& q,
    const Eigen::Matrix<double, 3, 1>& w)
{
    Eigen::Matrix<double, 4, 4> Omega;
    Omega(0, 0) = 0.0;
    Omega.block<3, 3>(1, 1) = hat(w);
    Omega.block<3, 1>(1, 0) = w.transpose();
    Omega.block<1, 3>(0, 1) = w;
    Eigen::Matrix<double, 4, 1> qv(q.w(), q.x(), q.y(), q.z());
    qv = 0.5 * Omega * qv;
    return Eigen::Quaternion<double>(qv(0), qv(1), qv(2), qv(3));
}

static Eigen::Quaternion<double> get_quaternion_error(
    const Eigen::Quaternion<double>& q,
    const Eigen::Quaternion<double>& qd)
{
    return qd.inverse() * q;
}

static Eigen::Matrix<double, 3, 3> hat(
    const Eigen::Matrix<double, 3, 1>& v)
{
    Eigen::Matrix<double, 3, 3> mat;
    mat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return mat;
}

#endif