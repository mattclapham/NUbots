#ifndef UTILITY_MATH_STATISTICS_STATISTICS_H
#define UTILITY_MATH_STATISTICS_STATISTICS_H

#include <armadillo>
#include <cmath>
#include <vector>

namespace utility {
namespace math {
    namespace statistics {
        // Calculate the mean of the data points.
        template <int N = 2>
        Eigen::Matrix<double, N, 1> calculateMean(const std::vector<Eigen::Matrix<double, N, 1>>& points) {
            Eigen::Matrix<double, N, 1> mean = Eigen::Matrix<double, N, 1>::Zero();

            for (const auto& point : points) {
                mean += point;
            }

            return (mean / points.size());
        }

        // Create a covariance matrix for all points in the window.
        // https://en.wikipedia.org/wiki/Covariance_matrix#Generalization_of_the_variance
        template <int N = 2>
        Eigen::Matrix<double, N, N> calculateCovarianceMatrix(const std::vector<Eigen::Matrix<double, N, 1>>& points,
                                                              const Eigen::Matrix<double, N, 1>& mean) {
            Eigen::Matrix<double, N, N> covariance = Eigen::Matrix<double, N, N>::Zero();

            for (const auto& point : points) {
                Eigen::Matrix<double, N, 1> offset = point - mean;
                covariance += offset * offset.transpose();
            }

            return (covariance / (points.size() - 1));
        }

        // Create a correlation matrix for all points in the window.
        // https://en.wikipedia.org/wiki/Covariance_matrix#Correlation_matrix
        template <int N = 2>
        Eigen::Matrix<double, N, N> calculateCorrelationMatrix(const Eigen::Matrix<double, N, N>& covariance) {
            Eigen::Matrix<double, N, N> diag = covariance.asDiagonal();
            diag = diag.unaryExpr([](double val) -> double {
                if (val > 0.0) {
                    return 1.0 / std::sqrt(val);
                }

                return val;
            });

            return (diag * covariance * diag);
        }
    }  // namespace statistics
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_STATISTICS_STATISTICS_H
