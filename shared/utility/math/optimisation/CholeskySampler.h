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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_OPTIMISATION_CHOLESKYSAMPLER_H
#define UTILITY_MATH_OPTIMISATION_CHOLESKYSAMPLER_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cmath>

#include "message/support/optimisation/OptimiserTypes.h"

#include "utility/support/eigen.h"

namespace utility {
namespace math {
    namespace optimisation {
        using message::support::optimisation::OptimiserParameters;
        using message::support::optimisation::OptimiserEstimate;

        class CholeskySampler {
        private:
            uint64_t batchSize;
            uint64_t sampleCount = 0;
            int generation       = -1;
            Eigen::VectorXd upperBound;
            Eigen::VectorXd lowerBound;
            Eigen::MatrixXd samples;

        public:
            CholeskySampler(const OptimiserParameters& params)
                : batchSize(params.batchSize)
                , generation(params.initial.generation)
                , upperBound(params.upperBound)
                , lowerBound(params.lowerBound)
                , samples() {}

            void clear() {
                generation = -1;
            }

            Eigen::MatrixXd getSamples(const OptimiserEstimate& bestParams, uint64_t numSamples) {
                if (bestParams.generation != generation || sampleCount + numSamples > batchSize) {
                    Eigen::MatrixXd projection = bestParams.covariance.llt().matrixL();
                    samples =
                        (utility::support::randn<Eigen::MatrixXd>(bestParams.estimate.size(), batchSize) * projection)
                            .transpose()
                            .colwise()
                        + bestParams.estimate;

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
                            utility::support::indexRows(samples, utility::support::find(outOfBounds.cwiseEqual(0)));

                        while (static_cast<uint64_t>(samples.rows()) < batchSize) {
                            Eigen::MatrixXd samples2 =
                                utility::support::randn<Eigen::MatrixXd>(bestParams.estimate.size(), batchSize);
                            samples2 = (samples2 * projection).transpose().colwise() + bestParams.estimate;

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
                            samples2 = utility::support::indexRows(samples2,
                                                                   utility::support::find(outOfBounds.cwiseEqual(0)));

                            samples << samples, samples2;
                        }

                        if (static_cast<uint64_t>(samples.cols()) >= batchSize) {
                            samples = samples.topRows(batchSize);
                        }
                    }

                    // reset required variables
                    sampleCount = 0;
                }

                sampleCount += numSamples;
                return samples.middleCols(sampleCount - numSamples, numSamples);
            }
        };
    }  // namespace optimisation
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_OPTIMISATION_CHOLESKYSAMPLER_H
