#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#pragma once
#include "kinematics.h"
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

static Eigen::Matrix<double, 3, 1> quaternion_feedback(
    const Eigen::Quaternion<double>& qe,
    const Eigen::Matrix<double, 3, 1>& w,
    const double& K,
    const double& C)
{
    return -K * qe.vec() * qe.w() - C * w;
}
}

#endif