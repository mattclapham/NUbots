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

#ifndef MODULES_ROBOTX_BEHAVIOUR_H
#define MODULES_ROBOTX_BEHAVIOUR_H

#define __USE_SINGLE_PRECISION__

#include <nuclear>
#include <NURobotX/Data/Types.h>
#include <NURobotX/Guidance/TrajectoryPlanner.h>
#include <eigen3/Eigen/Core>
#include <NURobotX/Data/VehicleState.h>

namespace modules {
namespace robotx {

    class Behaviour : public NUClear::Reactor {

    private:
        bool is_initialised;
        bool reported = false;
        bool run_autonomous;
        NURobotX::TrajectoryPlanner trajectory_planner;
        std::vector<std::vector<Eigen::Matrix2Xi>> task_paths;
        int path_test_tolerance;
        int current_task = 0;
        int current_path = 0;
        int path_timeout = 120;
        int path_report_time = 5;
        time_t path_start_time;

        bool goalReached(const NURobotX::VehicleState& state);
    public:
        /// @brief Called by the powerplant to build and setup the Behaviour reactor.
        explicit Behaviour(std::unique_ptr<NUClear::Environment> environment);
        static constexpr const char* CONFIGURATION_PATH = "Behaviour.yaml";
    };

}
}


#endif
