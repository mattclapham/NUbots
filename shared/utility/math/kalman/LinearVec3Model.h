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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_KALMAN_LINEARVEC3MODEL_H
#define UTILITY_MATH_KALMAN_LINEARVEC3MODEL_H

/* Inertial Motion Unit*/
namespace utility {
    namespace math {
        namespace kalman {

            class LinearVec3Model {
                // Number of dimensions
                // State is velocity of torso relative to the robot local ground matrix
                //
            public:
                static constexpr size_t size = 3;

                LinearVec3Model() {} // empty constructor

                Eigen::Matrix<double, size, 1> timeUpdate(const Eigen::Matrix<double, size, 1>& state, double deltaT, const Eigen::Vector3d& measurement);

                arma::vec predictedObservation(const Eigen::Matrix<double, size, 1>& state, std::nullptr_t);

                arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

                Eigen::Matrix<double, size, 1> limitState(const Eigen::Matrix<double, size, 1>& state);

                arma::mat::fixed<size, size> processNoise();

                const double processNoiseFactor = 1e-6;
            };


        }
    }
}
#endif
