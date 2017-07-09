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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "DarwinVirtualLoadSensor.h"
#include <nuclear>

namespace module {
namespace platform {
    namespace darwin {
        DarwinVirtualLoadSensor::DarwinVirtualLoadSensor()
            : noiseFactor(0.0)
            , currentNoise(2.0 * noiseFactor)
            , certaintyThreshold(0.0)
            , uncertaintyThreshold(0.0)
            , hiddenWeights()
            , hiddenBias()
            , outputWeights()
            , outputBias() {}

        DarwinVirtualLoadSensor::DarwinVirtualLoadSensor(const Eigen::MatrixXd& hiddenWeights,
                                                         const Eigen::VectorXd& hiddenBias,
                                                         const Eigen::MatrixXd& outputWeights,
                                                         const Eigen::VectorXd& outputBias,
                                                         double noiseFactor,
                                                         double certaintyThreshold,
                                                         double uncertaintyThreshold)
            : noiseFactor(noiseFactor)
            , currentNoise(2.0 * noiseFactor)
            , certaintyThreshold(certaintyThreshold)
            , uncertaintyThreshold(uncertaintyThreshold)
            , hiddenWeights(hiddenWeights)
            , hiddenBias(hiddenBias)
            , outputWeights(outputWeights)
            , outputBias(outputBias) {}

        bool DarwinVirtualLoadSensor::updateFoot(const Eigen::VectorXd& features) {

            Eigen::VectorXd result =
                ((hiddenWeights * features + hiddenBias)
                         .unaryExpr([](double elem) -> double {
                             return (std::max(0.0, std::min(elem, std::numeric_limits<double>::max())));
                         })
                         .transpose()
                     * outputWeights
                 + outputBias);

            double linResult = std::tanh(result[0] * 0.5) * 0.5 + 0.5;

            // do the bayes update (1D kalman filter thing)
            double k = currentNoise / (currentNoise + noiseFactor);
            state += k * (linResult - state);
            currentNoise *= 1.0 - k;
            currentNoise += 1.0;

            if (state >= certaintyThreshold) {
                outputState = true;
            }
            else if (state < uncertaintyThreshold) {
                outputState = false;
            }

            return outputState;
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
