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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_OPTIMISATION_GAUSSIANSAMPLER_H
#define UTILITY_MATH_OPTIMISATION_GAUSSIANSAMPLER_H

#include <Eigen/Core>
#include <cmath>

#include "message/support/optimisation/OptimiserTypes.h"

#include "utility/support/eigen.h"

namespace utility {
namespace math {
    namespace optimisation {
        using message::support::optimisation::OptimiserParameters;
        using message::support::optimisation::OptimiserEstimate;

        class GaussianSampler {
        private:
            uint64_t batchSize;
            uint64_t sampleCount = 0;
            int64_t generation   = -1;
            Eigen::VectorXd upperBound;
            Eigen::VectorXd lowerBound;
            Eigen::MatrixXd samples;

        public:
            GaussianSampler(const OptimiserParameters& params)
                : batchSize(params.batchSize)
                , generation(params.initial.generation)
                , upperBound(params.upperBound)
                , lowerBound(params.lowerBound)
                , samples() {}

            void clear() {
                generation = -1;
            }

            Eigen::MatrixXd getSamples(OptimiserEstimate& bestParams, uint64_t numSamples) {
                // note: bestParams.covariance is possibly mutable in this step, do not const it!
                if (bestParams.generation != generation || sampleCount + numSamples > batchSize
                    || samples.cols() == 0) {

                    // generate initial data
                    Eigen::VectorXd weights = bestParams.covariance.asDiagonal();
                    samples                 = utility::support::randn(bestParams.estimate.size(), batchSize);
                    samples                 = samples.colwise().cwiseProduct(weights);
                    samples.colwise() += bestParams.estimate;


                    // out of bounds check
                    if (lowerBound.size() > 0 and upperBound.size() > 0) {
                        Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> outOfBounds =
                            (samples.array() > upperBound.replicate(1, samples.cols()).array())
                                .select(samples, Eigen::MatrixXd::Zero(bestParams.estimate.size(), batchSize))
                                .rowwise()
                                .sum();
                        outOfBounds +=
                            (samples.array() < lowerBound.replicate(1, samples.cols()).array())
                                .select(samples, Eigen::MatrixXd::Zero(bestParams.estimate.size(), batchSize))
                                .rowwise()
                                .sum();
                        samples = utility::support::index(samples.colwise(), utility::support::find(outOfBounds == 0u));

                        while (static_cast<uint64_t>(samples.cols()) < batchSize) {
                            Eigen::MatrixXd samples2 = utility::support::randn(bestParams.estimate.size(), batchSize);
                            samples2                 = samples2.colwise().cwiseProduct(weights);
                            samples2.colwise() += bestParams.estimate;

                            outOfBounds =
                                (samples2.array() > upperBound.replicate(1, samples2.cols()).array())
                                    .select(samples2, Eigen::MatrixXd::Zero(bestParams.estimate.size(), batchSize))
                                    .rowwise()
                                    .sum();
                            outOfBounds +=
                                (samples2.array() < lowerBound.replicate(1, samples2.cols()).array())
                                    .select(samples2, Eigen::MatrixXd::Zero(bestParams.estimate.size(), batchSize))
                                    .rowwise()
                                    .sum();
                            samples2 =
                                utility::support::index(samples2.colwise(), utility::support::find(outOfBounds == 0u));

                            if (samples2.rows() > 0) {
                                samples << samples, samples2;
                            }
                        }

                        if (static_cast<uint64_t>(samples.size()) >= batchSize) {
                            samples = samples.leftCols(batchSize);
                        }
                    }

                    // reset required variables
                    sampleCount           = 0;
                    bestParams.covariance = weights.asDiagonal();
                }

                sampleCount += numSamples;
                return samples.middleCols(sampleCount - numSamples, numSamples);
            }
        };
    }
}
}

#endif  // UTILITY_MATH_OPTIMISATION_GAUSSIANSAMPLER_H
