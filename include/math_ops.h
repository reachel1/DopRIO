#ifndef RISE_MATH_OPS_H
#define RISE_MATH_OPS_H

#include <boost/array.hpp>
#include <boost/math/tools/precision.hpp>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <opencv2/core.hpp>

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

/// Commonly used mathematical functions
namespace rise::math {

// Constant definition
constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI;  // rad -> deg
constexpr double G_m_s2 = 9.81;            // Gravity

// Illegal definition
constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();

/**
 * Calculate the mean and diagonal covariance of data in a container
 * @tparam C    Container type
 * @tparam D    Result type
 * @tparam Getter 
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}

/**
 * Calculate the mean and matrix covariance of data in a container
 * @tparam C    Container type
 * @tparam int 　Data dimension
 * @tparam Getter  
 */
template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& data, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = data.size();
    assert(len > 1);

    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                          [&mean, &getter](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum + v * v.transpose();
                          }) / (len - 1);
    // clang-format on
}

/**
 * Gets the antisymmetric matrix
 * @tparam T  data type
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const T& v1, const T& v2, const T& v3) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v3, v2, v3, 0.0, -v1, -v2, v1, 0.0;
    return m;
}

}  // namespace rise::math

#endif  // RISE_MATH_OPS_H