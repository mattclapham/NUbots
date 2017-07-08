/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2014 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_PGAOPTIMISER_H
#define UTILITY_MATH_PGAOPTIMISER_H

#include <Eigen/Core>
#include <cmath>

#include "utility/support/eigen.h"

namespace utility {
namespace math {
    namespace optimisation {
        namespace PGA {
            /**
             * Function to generate a new best-estimate using the parameter samples and fitnesses provided.
             *
             * @param samples - the tested samples in the matrix format returned by getSamples below (one sample per
             * row)
             * @param fitnesses - a vector of fitnesses corresponding to each sample
             * @param c - a smoothing parameter (7 is almost universally acceptable so only change in extreme
             * circumstances)
             *
             * @returns bestEstimate - an updated best parameter estimate vector to re-sample from
             *
             * Takes a row-wise list of sample parameters, a corresponding vector of fitnesses, and a selectivity
             * constant c (don't change c unless you know what it does)
             * @author Josiah Walker
             */
            inline Eigen::VectorXd updateEstimate(const Eigen::MatrixXd& samples,
                                                  const Eigen::VectorXd& fitnesses,
                                                  const double c = 7.0) {

                // create a vector of normed fitnesses
                const double min                      = fitnesses.minCoeff();
                const double max                      = fitnesses.minCoeff();
                const Eigen::VectorXd normedFitnesses = (max * Eigen::VectorXd::Ones(fitnesses.size()) - fitnesses)
                                                        / (max - min + std::numeric_limits<double>::epsilon());

                // create a set of weights per sample which specifies the likelihood that they are near the best
                // estimate
                const Eigen::VectorXd sampleWeights = (-c * normedFitnesses).array().exp();

                // return the probabilistically weighted result estimate
                return samples.cwiseProduct((sampleWeights / sampleWeights.sum()).replicate(1, samples.cols()))
                    .colwise()
                    .sum()
                    .transpose();
            }

            /**
             * Function to create a new set of parameter samples from a best estimate and
             * a vec of per-dimension scales for the gaussian noise additive (sigmaweights).
             * @param bestEstimate - the current estimated best parameter set
             *
             * @param sigmaWeights - the variance in each parameter dimension for drawing samples from (ie the scale of
             * each dimension)
             * @param numSamples - the number of test parameter sets to generate (this varies, but 5-12 is usually good)
             *
             * @returns newSamples - a numSamples rows by numParams cols matrix of sample parameter sets to try
             *
             * @author Josiah walker
             */
            inline Eigen::MatrixXd getSamples(const Eigen::VectorXd& bestEstimate,
                                              const Eigen::VectorXd& sigmaWeights,
                                              const size_t& numSamples) {
                return utility::support::randn(numSamples, bestEstimate.size())
                           .cwiseProduct(sigmaWeights.replicate(1, numSamples).transpose())
                       + bestEstimate.replicate(1, numSamples).transpose();
            }
        }
    }
}
}


#endif  // UTILITY_MATH_COORDINATES_H
