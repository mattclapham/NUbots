/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_FILTER_UKF_H
#define UTILITY_MATH_FILTER_UKF_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <nuclear>

#include "utility/support/LazyEvaluation.h"

namespace utility {
namespace math {
    namespace filter {

        template <typename Model>  // model is is a template parameter that Kalman also inherits
        class UKF {
        public:
            // The model
            Model model;

            // Dimension types for vectors and square matricies
            using StateVec = Eigen::Matrix<double, Model::size, 1>;
            using StateMat = Eigen::Matrix<double, Model::size, Model::size>;

        private:
            // The number of sigma points
            static constexpr uint NUM_SIGMA_POINTS = (Model::size * 2) + 1;

            using SigmaVec       = Eigen::Matrix<double, NUM_SIGMA_POINTS, 1>;
            using SigmaRowVec    = Eigen::Matrix<double, 1, NUM_SIGMA_POINTS>;
            using SigmaMat       = Eigen::Matrix<double, Model::size, NUM_SIGMA_POINTS>;
            using SigmaSquareMat = Eigen::Matrix<double, NUM_SIGMA_POINTS, NUM_SIGMA_POINTS>;

            // Our estimate and covariance
            StateVec mean;
            StateMat covariance;

            // Our sigma points for UKF
            StateVec sigmaMean;
            SigmaMat sigmaPoints;

            SigmaMat centredSigmaPoints;  // X in Steves kalman theory
            SigmaVec d;
            SigmaSquareMat covarianceUpdate;  // C in Steves kalman theory

            SigmaSquareMat defaultCovarianceUpdate;

            // The mean and covariance weights
            SigmaVec meanWeights;
            SigmaRowVec covarianceWeights;

        private:
            // UKF variables
            double covarianceSigmaWeights;

            void generateSigmaPoints(SigmaMat& points, const StateVec& mean, const StateMat& covariance) {

                // Our first row is always the mean
                points.col(0) = mean;

                // Get our cholesky decomposition
                Eigen::MatrixXd chol = (covarianceSigmaWeights * covariance).llt().matrixL();

                // Put our values in either end of the matrix
                for (uint i = 1; i < Model::size + 1; ++i) {
                    Eigen::VectorXd deviation   = chol.col(i - 1);
                    points.col(i)               = (mean + deviation);
                    points.col(i + Model::size) = (mean - deviation);
                }
            }

            void meanFromSigmas(StateVec& mean, const SigmaMat& sigmaPoints) const {
                mean = sigmaPoints * meanWeights;
            }

            void covarianceFromSigmas(StateMat& covariance, const SigmaMat& sigmaPoints, const StateVec& mean) const {

                SigmaMat meanCentered{sigmaPoints - mean.template replicate<1, NUM_SIGMA_POINTS>()};
                covariance = covarianceWeights.template replicate<Model::size, 1>().cwiseProduct(meanCentered)
                             * meanCentered.transpose();
            }

            void meanFromSigmas(Eigen::VectorXd& mean, const Eigen::MatrixXd& sigmaPoints) const {
                mean = sigmaPoints * meanWeights;
            }

            void covarianceFromSigmas(Eigen::MatrixXd& covariance,
                                      const Eigen::MatrixXd& sigmaPoints,
                                      const Eigen::VectorXd& mean) const {

                Eigen::MatrixXd meanCentered = sigmaPoints - mean.replicate(1, NUM_SIGMA_POINTS);
                covariance =
                    covarianceWeights.replicate(mean.size(), 1).cwiseProduct(meanCentered) * meanCentered.transpose();
            }

        public:
            UKF(StateVec initialMean       = StateVec::Zero(),
                StateMat initialCovariance = StateMat::Identity() * 0.1,
                double alpha               = 1e-1,
                double kappa               = 0.f,
                double beta                = 2.f)
                : model()
                , mean(StateVec::Zero())
                , covariance(StateMat::Identity())
                , sigmaMean(StateVec::Zero())
                , sigmaPoints(SigmaMat::Zero())
                , centredSigmaPoints(SigmaMat::Zero())
                , d(SigmaVec::Zero())
                , covarianceUpdate(SigmaSquareMat::Identity())
                , defaultCovarianceUpdate(SigmaSquareMat::Identity())
                , meanWeights(SigmaVec::Zero())
                , covarianceWeights(SigmaRowVec::Zero())
                , covarianceSigmaWeights(0.0) {

                reset(initialMean, initialCovariance, alpha, kappa, beta);
            }

            void reset(StateVec initialMean, StateMat initialCovariance, double alpha, double kappa, double beta) {
                double lambda = (alpha * alpha) * (Model::size + kappa) - Model::size;

                covarianceSigmaWeights = Model::size + lambda;

                meanWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                meanWeights[0] = lambda / (Model::size + lambda);

                covarianceWeights.fill(1.0 / (2.0 * (Model::size + lambda)));
                covarianceWeights[0] = lambda / (Model::size + lambda) + (1.0 - pow(alpha, 2) + beta);

                defaultCovarianceUpdate = covarianceWeights.asDiagonal();

                setState(initialMean, initialCovariance);
            }

            void setState(StateVec initialMean, StateMat initialCovariance) {
                mean       = initialMean;
                covariance = initialCovariance;

                // Calculate our sigma points
                sigmaMean = mean;
                generateSigmaPoints(sigmaPoints, mean, covariance);

                // Reset our state for more measurements
                covarianceUpdate = defaultCovarianceUpdate;
                d.setZero();
                centredSigmaPoints = sigmaPoints - sigmaMean.template replicate<1, NUM_SIGMA_POINTS>();
            }

            template <typename... TAdditionalParameters>
            void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters) {
                // Generate our sigma points
                generateSigmaPoints(sigmaPoints, mean, covariance);

                // Write the propagated version of the sigma point
                for (uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                    sigmaPoints.col(i) = model.timeUpdate(sigmaPoints.col(i), deltaT, additionalParameters...);
                }

                // Calculate the new mean and covariance values.
                meanFromSigmas(mean, sigmaPoints);
                mean = model.limitState(mean);
                covarianceFromSigmas(covariance, sigmaPoints, mean);
                covariance += model.processNoise();

                // Re calculate our sigma points
                sigmaMean = mean;
                generateSigmaPoints(sigmaPoints, mean, covariance);

                // Reset our state for more measurements
                covarianceUpdate = defaultCovarianceUpdate;
                d.setZero();
                centredSigmaPoints = sigmaPoints - sigmaMean.template replicate<1, NUM_SIGMA_POINTS>();
            }

            template <typename TMeasurement, typename... TMeasurementArgs>
            utility::support::LazyEvaluation<double> measurementUpdate(const TMeasurement& measurement,
                                                                       const Eigen::MatrixXd& measurementVariance,
                                                                       const TMeasurementArgs&... measurementArgs) {

                // Allocate room for our predictions
                Eigen::MatrixXd predictedObservations(measurement.size(), NUM_SIGMA_POINTS);

                // First step is to calculate the expected measurement for each sigma point.
                for (uint i = 0; i < NUM_SIGMA_POINTS; ++i) {
                    predictedObservations.col(i) = model.predictedObservation(sigmaPoints.col(i), measurementArgs...);
                }

                // Now calculate the mean of these measurement sigmas.
                Eigen::VectorXd predictedMean;
                meanFromSigmas(predictedMean, predictedObservations);
                Eigen::MatrixXd centredObservations =
                    predictedObservations - predictedMean.template replicate<1, NUM_SIGMA_POINTS>();

                // Update our state
                covarianceUpdate -=
                    covarianceUpdate.transpose() * centredObservations.transpose()
                    * (measurementVariance + centredObservations * covarianceUpdate * centredObservations.transpose())
                          .inverse()
                    * centredObservations * covarianceUpdate;

                // In file included from /nubots/toolchain/native/include/eigen3/Eigen/Core:347:0,
                //                  from ../module/platform/darwin/SensorFilter/src/SensorFilter.h:23,
                //                  from ../module/platform/darwin/SensorFilter/src/SensorFilter.cpp:20:

                // In instantiation of ‘const Eigen::Product<Derived, OtherDerived>
                // Eigen::MatrixBase<Derived>::operator*(const Eigen::MatrixBase<OtherDerived>&) const
                // [with OtherDerived = Eigen::Transpose<Eigen::Matrix<double, -1, 1> >; Derived =
                // Eigen::Transpose<Eigen::Matrix<double, 27, 27, 0, 27, 27> >]’:

                // UKF.h:213:50:
                // required from ‘utility::support::LazyEvaluation<double>
                // utility::math::filter::UKF<Model>::measurementUpdate(const TMeasurement&,
                //                                                                                                              const MatrixXd&,
                //                                                                                                              const TMeasurementArgs& ...)
                // [with TMeasurement = Eigen::Matrix<double, 3, 1>;
                //  TMeasurementArgs = {module::platform::darwin::MotionModel::MeasurementType::ACCELEROMETER};
                //  Model = module::platform::darwin::MotionModel; Eigen::MatrixXd = Eigen::Matrix<double, -1, -1>]’

                // SensorFilter.cpp:529:70:
                // required from here
                // /nubots/toolchain/native/include/eigen3/Eigen/src/Core/util/StaticAssert.h:32:40: error: static
                // assertion failed: INVALID_MATRIX_PRODUCT


                const Eigen::MatrixXd innovation = model.observationDifference(measurement, predictedMean);
                d += (centredObservations.transpose()) * measurementVariance.inverse() * innovation;

                // Update our mean and covariance
                mean       = sigmaMean + centredSigmaPoints * covarianceUpdate * d;
                mean       = model.limitState(mean);
                covariance = centredSigmaPoints * covarianceUpdate * centredSigmaPoints.transpose();

                // Calculate and return the likelihood of the prior mean
                // and covariance given the new measurement (i.e. the
                // prior probability density of the measurement):
                return utility::support::LazyEvaluation<double>(
                    [this, predictedObservations, predictedMean, measurementVariance, innovation] {
                        Eigen::MatrixXd predictedCovariance;

                        covarianceFromSigmas(predictedCovariance, predictedObservations, predictedMean);

                        Eigen::MatrixXd innovationVariance = predictedCovariance + measurementVariance;
                        Eigen::MatrixXd scalarlikelihoodExponent =
                            ((innovation.transpose() * innovationVariance.inverse()) * innovation);

                        double loglikelihood =
                            0.5 * (std::log(innovationVariance.determinant()) + std::abs(scalarlikelihoodExponent(0))
                                   + innovation.size() * std::log(2 * M_PI));

                        return -loglikelihood;
                    });
            }

            const StateVec& get() const {
                return mean;
            }

            const StateMat& getCovariance() const {
                return covariance;
            }
        };
    }
}
}


#endif
