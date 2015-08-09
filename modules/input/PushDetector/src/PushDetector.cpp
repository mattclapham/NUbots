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

#include "PushDetector.h"

#include "messages/support/Configuration.h"
#include "utility/time/time.h"
#include "utility/nubugger/NUhelpers.h"
#include "messages/input/Sensors.h"
#include "messages/input/PushDetection.h"

namespace modules {
namespace input {

    using messages::support::Configuration;
    using messages::input::Sensors;
    using messages::input::PushDetection;
	using utility::math::kalman::UKF;
	using utility::time::TimeDifferenceSeconds;
    using utility::nubugger::graph;

    PushDetector::PushDetector(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<Configuration<PushDetector>>>([this] (const Configuration<PushDetector>& config) {
            // Use configuration here from file PushDetector.yaml

        	// Initialise the vector of load filters:
        	for (int i = 0; i < 20; i++) {
        		auto filter = UKF<ServoLoadModel>(
	        		{0}, // mean
	                arma::eye(ServoLoadModel::size, ServoLoadModel::size) * 1, // cov
	                0.1); // alpha
        		loadFilters.push_back(filter);
        	}

        	lastTimeUpdateTime = NUClear::clock::now();
        });

        on<Trigger<Last<2, Sensors>>>([this] (const LastList<Sensors>& sensors) {
        	NUClear::log("before");
        	if (sensors.size() < 2) {
        		return;
        	}
        	NUClear::log("after");


        	arma::vec3 diff = sensors[0]->accelerometer - sensors[1]->accelerometer;
        	arma::vec2 xzDiff = { diff(0), diff(2) };
        	
        	if (arma::norm(xzDiff) > 5) {
        		PushDetection det;
        		det.forward = xzDiff(0) > 0;
        		emit(std::make_unique<PushDetection>(det));
    			emit(graph("PD: Detection", xzDiff(0) > 0 ? 1 : -1));
        	} else {
    			emit(graph("PD: Detection", 0));
        	}

        	// Load filtering:
   //  		auto currentTime = NUClear::clock::now();
   //  		double seconds = TimeDifferenceSeconds(currentTime, lastTimeUpdateTime);
			// lastTimeUpdateTime = currentTime;

   //      	for (int i = 0; i < loadFilters.size(); i++) {
   //  			auto& filter = loadFilters[i];

   //  			filter.timeUpdate(seconds);
   //  			arma::mat cov = { 0.1 };
   //  			arma::vec meas = { sensors.servos[i].load };
   //      		float likelihood = filter.measurementUpdate(meas, cov);
   //      	}
		
			// // Output filtered values to NUsight:
			// arma::vec::fixed<20> filteredLoads;
   //  		for (int i = 0; i < loadFilters.size(); i++) {
   //  			filteredLoads(i) = loadFilters[i].get()(0);
   //  		}
   //  		emit(graph("PD: Filtered Loads", filteredLoads));
   //  		NUClear::log("Print the data!");


        });
    }
}
}