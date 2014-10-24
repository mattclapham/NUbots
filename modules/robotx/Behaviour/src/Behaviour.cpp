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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Behaviour.h"
#include "messages/support/Configuration.h"
#include <armadillo>
#include <iostream>
#include "utility/support/armayamlconversions.h"
#include "messages/robotx/AutonomousMode.h"
#include "messages/robotx/ControlReference.h"
#include "messages/input/RobotXState.h"

namespace modules {
    namespace robotx {

        using messages::support::Configuration;
        using messages::robotx::AutonomousMode;
        using messages::robotx::ControlReference;
        using messages::input::RobotXState;
        using namespace NURobotX;

        Behaviour::Behaviour(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , is_initialised(false){

            on<Trigger<Configuration<Behaviour>>>([this] (const Configuration<Behaviour>& file) {
                int max_velocity = file.config["maxVelocity"].as<int>();
                std::cout << "max velocity: "<< max_velocity << std::endl;

                double line_of_sight = file.config["lineOfSightMeters"].as<double>();
                std::cout << "line of sight: " << line_of_sight << std::endl;

                path_test_tolerance = file.config["testPathGoalTolerance"].as<int>();
                std::cout << "path test tolerance: " << path_test_tolerance << std::endl;

                std::cout << "Path Test: " << std::endl;
                for (const auto& task : file.config["tasks"]) {
                    task_paths.push_back({});
                    std::cout << task.first << std::endl;
                    for (const auto& path : task.second) {
                    
                        Eigen::Matrix2Xi path_test;
                        path_test.conservativeResize(2,path.size());
                        for(uint i =0; i < path.size(); i++) {
                            path_test(0,i) = path[i].as<arma::vec>()[0];
                            path_test(1,i) = path[i].as<arma::vec>()[1];
                        }
                        std::cout << path_test << std::endl;
                        task_paths.back().push_back(path_test);
                    }
                }

                trajectory_planner = TrajectoryPlanner(max_velocity, line_of_sight);
                is_initialised = true;
                std::cout << "Trajectory planner initialised" << std::endl;
            });

            on<Trigger<AutonomousMode>>([this] (const AutonomousMode& mode) {
                run_autonomous = mode.on;
            });

            on<Trigger<RobotXState>>([this](const RobotXState& state)
            {
                if(run_autonomous) {
                    VehicleState vehicle_state;
                    vehicle_state.mean() = Vector15s::Map(state.state.memptr());
                    vehicle_state.covariance() = Matrix15s::Map(state.covariance.memptr());
                    vehicle_state.time_stamp = state.timestamp;

                    if(!goalReached(vehicle_state)) {
                        auto ref = trajectory_planner.getHeadingVelocity(vehicle_state, task_paths[current_task][current_path]);

                        auto control_ref = std::make_unique<ControlReference>();
                        control_ref->heading = ref(0);
                        control_ref->velocity = ref(1);

                        emit(std::move(control_ref));
                     } else {
                        
                        ++current_path;
                        
                        if (current_path >= int(task_paths[current_task].size())) {
                            current_path = 0;
                            ++current_task;
                            current_task %= task_paths.size();
                        }
                        //XXX: emit task change
                        
                        
                        //XXX: save the current time for a timeout
                     
                     }
                }
            });
        }

        bool Behaviour::goalReached(const VehicleState& state)
        {
            float N_goal = static_cast<float>(task_paths[current_path][current_task](0,task_paths[current_task][current_path].cols()-1));
            float E_goal = static_cast<float>(task_paths[current_path][current_task](1,task_paths[current_task][current_path].cols()-1));

            auto rBNn = state.rBNn();

            float N = rBNn[0];
            float E = rBNn[1];

            if ( (N <= N_goal + path_test_tolerance || N >= N_goal - path_test_tolerance)
                 && (E <= E_goal + path_test_tolerance || E >= E_goal - path_test_tolerance)) {
                return true;
            }
            return false;
        }
    }
}

