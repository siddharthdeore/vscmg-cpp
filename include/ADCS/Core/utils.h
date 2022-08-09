#ifndef _ADCS_CORE_UTILS_H
#define _ADCS_CORE_UTILS_H

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <math.h>

namespace utils {
using namespace Eigen;

/**
 * @brief Returns vector containig max value among two input vectors
 *
 * @tparam Derived
 * @tparam T
 * @param a
 * @param b
 * @return Derived
 */
template <typename Derived, typename T>
inline Derived max(const MatrixBase<Derived>& a, T b)
{
    return a.array().max(b);
}

/**
 * @brief Returns max value given two variables
 *
 * @tparam T
 * @tparam std::enable_if_t<std::is_scalar<T>::value>
 * @param a
 * @param b
 * @return T
 */
template <typename T, typename = typename std::enable_if_t<std::is_scalar<T>::value>>
inline T max(T a, T b)
{
    return std::max(a, b);
}

/**
 * @brief Returns min value given two variables
 *
 * @tparam T
 * @tparam std::enable_if_t<std::is_scalar<T>::value>
 * @param a
 * @param b
 * @return T
 */
template <typename T, typename = typename std::enable_if_t<std::is_scalar<T>::value>>
inline T min(T a, T b)
{
    return std::min(a, b);
}

/**
 * @brief Clamps two input between min and max value.
 *
 * @tparam T
 * @tparam std::enable_if_t<std::is_scalar<T>::value>
 * @param x
 * @param minVal
 * @param maxVal
 * @return T
 */
template <typename T, typename = typename std::enable_if_t<std::is_scalar<T>::value>>
inline T clamp(T x, T minVal, T maxVal)
{
    return min(max(x, minVal), maxVal);
}

/**
 * @brief Clamps two input Vector between min and max Vector.
 *
 * @param x
 * @param minVal
 * @param maxVal
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d clamp(Eigen::Vector3d x, Eigen::Vector3d minVal, Eigen::Vector3d maxVal)
{
    return Eigen::Vector3d(clamp(x.x(), minVal.x(), maxVal.x()), clamp(x.y(), minVal.y(), maxVal.y()), clamp(x.z(), minVal.z(), maxVal.z()));
}

/**
 * @brief Clamps two input Vector between min and max Vector.
 *
 * @param x
 * @param minVal
 * @param maxVal
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d clamp(const Eigen::Vector3d& x, const double minVal, const double maxVal)
{
    return Eigen::Vector3d(clamp(x.x(), minVal, maxVal), clamp(x.y(), minVal, maxVal), clamp(x.z(), minVal, maxVal));
}

/**
 * @brief Returns sign of input parameter.
 *
 * @tparam T
 * @tparam std::enable_if_t<std::is_scalar<T>::value>
 * @param val
 * @return constexpr T
 */
template <typename T, typename = typename std::enable_if_t<std::is_scalar<T>::value>>
[[nodiscard]] constexpr inline T sign(T val)
{
    return static_cast<T>((0 < val) - (val < 0));
}
}

#endif