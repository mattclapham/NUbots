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

#include "WalkFitness.h"
#include "messages/motion/GetupCommand.h"
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"
#include "messages/support/optimisation/WalkFitnessDelta.h"

namespace modules {
    namespace support {
        namespace optimisation {
        
            using messages::support::Configuration;
            using messages::motion::ExecuteGetup;
            using messages::input::Sensors;
            using messages::support::optimisation::WalkFitnessDelta;
        
            WalkFitness::WalkFitness(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {
        
                on<Trigger<ExecuteGetup>>([this](const ExecuteGetup&) {
                    emit(std::make_unique<WalkFitnessDelta>(-1));
                });
        
        
                //on<Trigger<Last<20,Sensors>>>([this](const LastList<Sensors>& sensors) {
                //	...
             //    emit(std::make_unique<WalkFitnessDelta>(1));
                //});
        
                // OnTiggerSensors
        
                //on<Trigger<Configuration<WalkFitness>>>([this] (const Configuration<WalkFitness>& config) {
                //    // Use configuration here from file WalkFitness.yaml
                //});
            }
        }
    }
}
