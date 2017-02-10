/*
 * This file is part of NUbots Codebase.
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

#ifndef MODULES_INPUT_SERVOLOADMODEL_H
#define MODULES_INPUT_SERVOLOADMODEL_H

#include <nuclear>
#include <armadillo>
#include <chrono>
#include <yaml-cpp/yaml.h>

namespace utility 
{
namespace input 
{
    class ServoLoadModel 
    {
       public:
        static constexpr size_t size = 1;

        ServoLoadModel() {} // empty constructor

        arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double /*deltaT*/) {
            return state;
        }

        arma::vec::fixed<size> predictedObservation(const arma::vec::fixed<size>& state) {
            return state;
        }

        arma::vec observationDifference(const arma::vec& a, const arma::vec& b) {
            return a - b;
        }

        arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state) {
            return state;
        }

        arma::mat::fixed<size, size> processNoise() {
            return arma::eye(ServoLoadModel::size, ServoLoadModel::size) * 0.001;
        }
    };
}
}

#endif  // MODULES_INPUT_SERVOLOADMODEL_H
