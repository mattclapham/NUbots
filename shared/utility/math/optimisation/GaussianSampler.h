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
                    // https://forum.kde.org/viewtopic.php?f=74&t=98568 (Coefficient-Wise Product and Broadcasting)
                    samples = (samples.array().colwise() * weights.array()).matrix();
                    samples.colwise() += bestParams.estimate;

                    // out of bounds check
                    if (lowerBound.size() > 0 && upperBound.size() > 0) {
                        Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> outOfBounds;
                        outOfBounds = ((samples - upperBound.replicate(samples.cols(), 1)).array() > 0.0)
                                          .matrix()
                                          .rowwise()
                                          .count()
                                          .cast<unsigned int>()
                                      + ((samples - lowerBound.replicate(samples.cols(), 1)).array() < 0.0)
                                            .matrix()
                                            .rowwise()
                                            .count()
                                            .cast<unsigned int>();
                        samples =
                            utility::support::indexCols(samples, utility::support::find(outOfBounds.cwiseEqual(0)));

                        while (static_cast<uint64_t>(samples.cols()) < batchSize) {
                            Eigen::MatrixXd samples2 = utility::support::randn(bestParams.estimate.size(), batchSize);
                            samples2                 = (samples2.array().colwise() * weights.array()).matrix();
                            samples2.colwise() += bestParams.estimate;

                            outOfBounds = ((samples2 - upperBound.replicate(samples2.cols(), 1)).array() > 0.0)
                                              .matrix()
                                              .rowwise()
                                              .count()
                                              .cast<unsigned int>()
                                          + ((samples2 - lowerBound.replicate(samples2.cols(), 1)).array() < 0.0)
                                                .matrix()
                                                .rowwise()
                                                .count()
                                                .cast<unsigned int>();
                            samples2 = utility::support::indexCols(samples2,
                                                                   utility::support::find(outOfBounds.cwiseEqual(0)));

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
