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

#include "ManualWalkOptimiser.h"

#include "messages/support/Configuration.h"

namespace modules {
namespace support {
namespace optimisation {

    using messages::support::Configuration;

    struct WalkEngineConfig {
        static constexpr const char* CONFIGURATION_PATH = "WalkEngine.yaml";
    };
    

    ManualWalkOptimiser::ManualWalkOptimiser(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), fitnessSum(0.0), currentSample(-1) {

        on<Trigger<Every<20, std::chrono::seconds>>
        , With<Configuration<ManualWalkOptimiser>>
        , With<Configuration<WalkEngineConfig>>([this] 
            (const time_t&
                , const Configuration<ManualWalkOptimiser>& optimiserConfig
                , const Configuration<WalkEngineConfig>& walkEngineConfig) {

            if (currentSample == -1) {
                numSamples             = optimiserConfig["number_of_samples"].as<uint>();
                numParameters          = optimiserConfig["number_of_parameters"].as<uint>();
                getUpCancelThreshold   = optimiserConfig["getup_cancel_trial_threshold"].as<uint>();
                configWaitMilliseconds = optimiserConfig["configuration_wait_milliseconds"].as<uint>();

                weights.set_size(numParameters);
                weights[0] = optimiserConfig["parameters_and_sigmas"]["stance"]["body_tilt"].as<double>();
                weights[1] = optimiserConfig["parameters_and_sigmas"]["walk_cycle"]["zmp_time"].as<double>();
                weights[2] = optimiserConfig["parameters_and_sigmas"]["walk_cycle"]["step_time"].as<double>();
                weights[3] = optimiserConfig["parameters_and_sigmas"]["walk_cycle"]["single_support_phase"]["start"].as<double>();
                weights[4] = optimiserConfig["parameters_and_sigmas"]["walk_cycle"]["single_support_phase"]["end"].as<double>();
                weights[5] = optimiserConfig["parameters_and_sigmas"]["walk_cycle"]["step"]["height"].as<double>();

                samples.set_size(numParameters, numSamples);
                samples.zeros();
                samples(0, 0) = walkEngineConfig["stance"]["body_tilt"].as<double>();
                samples(0, 1) = walkEngineConfig["walk_cycle"]["zmp_time"].as<double>();
                samples(0, 2) = walkEngineConfig["walk_cycle"]["step_time"].as<double>();
                samples(0, 3) = walkEngineConfig["walk_cycle"]["single_support_phase"]["start"].as<double>();
                samples(0, 4) = walkEngineConfig["walk_cycle"]["single_support_phase"]["end"].as<double>();
                samples(0, 5) = walkEngineConfig["walk_cycle"]["step"]["height"].as<double>();

                currentSample = numSamples;
                fitnessScores.zeros(numSamples);
            }
            
            if (currentSample == numSamples) {
                // Get the current best estimate.
                auto bestEstimate = updateEstimate(samples, fitnessScores);

                // Get the next set of samples.
                samples = getSamples(bestEstimate, weights, numSamples);

                // Start at the first sample.
                currentSample = 0;
            } 

            else {
                // Pair fitness score with sample.
                fitnessScores[currentSample] = fitnessSum;

                // Progress to next sample.
                currentSample++;
            }

            // Clear fitness score for next run.
            fitnessSum = 0.0;

            // Generate config file with next sample.
            walkEngineConfig["stance"]["body_tilt"]                         = samples(currentSample, 0);
            walkEngineConfig["walk_cycle"]["zmp_time"]                      = samples(currentSample, 1);
            walkEngineConfig["walk_cycle"]["step_time"]                     = samples(currentSample, 2);
            walkEngineConfig["walk_cycle"]["single_support_phase"]["start"] = samples(currentSample, 3);
            walkEngineConfig["walk_cycle"]["single_support_phase"]["end"]   = samples(currentSample, 4);
            walkEngineConfig["walk_cycle"]["step"]["height"]                = samples(currentSample, 5);

            // Save config file.
            Configuration::SaveConfiguration out;
            out.path   = "WalkEngine.yaml";
            out.config = walkEngineConfig.config;
            emit(std::move(std::make_unique<Configuration::SaveConfiguration>(out)));
        });

        on<Trigger<WalkFitnessDelta>>([this](const WalkFitnessDelta& delta) {
           fitnessSum += delta.fitnessDelta;
        });
    }
}
}
}
