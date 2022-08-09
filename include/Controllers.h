#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#pragma once
#include "kinematics.h"
#include "utils.h"
namespace Controller {

/**
 * @brief Generalized Quaternion Feedback Control law
 *
 * @param qe Error in desired and current orieantation
 * @param w Angular velocity of rigid body
 * @param K artificial stiffness coefficient
 * @param C damping
 * @return Eigen::Matrix<double, 3, 1>
 */
static Eigen::Matrix<double, 3, 1> quaternion_feedback(
    const Eigen::Quaternion<double>& qe,
    const Eigen::Matrix<double, 3, 1>& w,
    const double& K,
    const double& C);

/**
 * @brief quaternion feedback with discreate step output
 *
 * @param qe
 * @param w
 * @param K
 * @param C
 * @param C
 * @param step posible step of torque in Nm, default step value is resolution of 12 bit actuator of max 1 Nm
 * @return Eigen::Matrix<double, 3, 1>
 */
static Eigen::Matrix<double, 3, 1> quaternion_feedback_discreate(
    const Eigen::Quaternion<double>& qe,
    const Eigen::Matrix<double, 3, 1>& w,
    const double& K,
    const double& C,
    const double max_torque = 1.0,
    const double step = 0.000244140625);

static Eigen::Matrix<double, 3, 1> quaternion_feedback(
    const Eigen::Quaternion<double>& qe,
    const Eigen::Matrix<double, 3, 1>& w,
    const double& K,
    const double& C)
{
    return -K * qe.vec() * qe.w() - C * w;
}
static Eigen::Matrix<double, 3, 1> quaternion_feedback_discreate(
    const Eigen::Quaternion<double>& qe,
    const Eigen::Matrix<double, 3, 1>& w,
    const double& K,
    const double& C,
    const double max_torque,
    const double step)
{
    const Eigen::Matrix<double, 3, 1> u = utils::clamp(-K * qe.vec() * qe.w() - C * w, -max_torque, max_torque) / step;
    return u.array().round() * step;
}
}

#endif