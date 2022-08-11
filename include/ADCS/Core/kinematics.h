#ifndef KINEMATICS_H
#define KINEMATICS_H

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief Special Orthogonal Group
 *
 */
namespace so3 {
/**
 * @brief The hat operator takes a vector and transforms it into its equivalent matrix
 * to represent the cross product operation
 *
 * @param vec
 * @return Eigen::Matrix<double, 3, 3>
 */
static Eigen::Matrix<double, 3, 3> hat(
    const Eigen::Matrix<double, 3, 1>& vec);

static Eigen::Matrix<double, 3, 3> hat(
    const Eigen::Matrix<double, 3, 1>& vec)
{
    Eigen::Matrix<double, 3, 3> mat;
    mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return mat;
}
}
/**
 * @brief The quaternion error vector, The quaternion associated with
 * the rotation that takes desired frame (Fd) onto body frame (Fb),
 * the amplitude of which thus provides the misalignment error of body
 * frame (Fb) with respect to desired frame (Fd).
 * @param q
 * @param w
 * @return Eigen::Quaternion<double>
 */
static Eigen::Quaternion<double> get_quaternion_error(
    const Eigen::Quaternion<double>& q,
    const Eigen::Quaternion<double>& qd);

/**
 * @brief The time evolution of time varying quaternion q with angular velocity w
 *
 * @param q
 * @param w
 * @return Eigen::Quaternion<double>
 */
static Eigen::Quaternion<double> get_quaternion_kinematics(
    const Eigen::Quaternion<double>& q,
    const Eigen::Matrix<double, 3, 1>& w);

static Eigen::Quaternion<double> get_quaternion_kinematics(
    const Eigen::Quaternion<double>& q,
    const Eigen::Matrix<double, 3, 1>& w)
{
    Eigen::Matrix<double, 4, 4> Omega;
    Omega(0, 0) = 0.0;
    Omega.block<3, 3>(1, 1) = -so3::hat(w);
    Omega.block<3, 1>(1, 0) = w;
    Omega.block<1, 3>(0, 1) = -w.transpose();
    Eigen::Matrix<double, 4, 1> qv(q.w(), q.x(), q.y(), q.z());
    auto qvout = 0.5 * Omega * qv;
    return Eigen::Quaternion<double>(qvout(0), qvout(1), qvout(2), qvout(3));
}

static Eigen::Quaternion<double> get_quaternion_error(
    const Eigen::Quaternion<double>& q,
    const Eigen::Quaternion<double>& qd)
{
    return qd.inverse() * q;
}

#endif