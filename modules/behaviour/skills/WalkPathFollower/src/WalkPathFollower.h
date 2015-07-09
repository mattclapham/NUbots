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

#ifndef MODULES_BEHAVIOUR_SKILLS_WALKPATHFOLLOWER_H
#define MODULES_BEHAVIOUR_SKILLS_WALKPATHFOLLOWER_H

#include <nuclear>
#include "messages/behaviour/WalkPath.h"
#include "messages/motion/WalkCommand.h"
#include "utility/math/matrix/Transform2D.h"

namespace modules {
namespace behaviour {
namespace skills {

    using messages::behaviour::WalkPath;
    using messages::motion::WalkCommand;
    using utility::math::matrix::Transform2D;

    class WalkPathFollower : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the WalkPathFollower reactor.
        explicit WalkPathFollower(std::unique_ptr<NUClear::Environment> environment);

        /// @brief The instantaneous walk command required to start moving to the next unvisited node on the path.
        std::unique_ptr<WalkCommand> walkToNextNode(const Transform2D& currentState, bool noLogging = false);

        WalkCommand walkBetweenFar(const Transform2D& currentState, const Transform2D& targetState);

        WalkCommand walkBetweenNear(const Transform2D& currentState, const Transform2D& targetState);

        /// @brief Remove already visited states from the given path.
        /// Returns the number of states removed.
        int trimPath(const Transform2D& currentState, WalkPath& walkPath);

        /// @brief The index of the closest state in walkPath to currentState.
        int closestPathIndex(const Transform2D& currentState, const WalkPath& walkPath);

        /// @brief The path the robot is expected to follow while following the given path.
        WalkPath estimatedPath(const Transform2D& currentState, const WalkPath& walkPath, double timeStep, int simSteps, int sample);

        /// @brief Return whether currentState is close enough to visitState for us to say that the robot has 'visited' that state.
        bool isVisited(const Transform2D& currentState, const Transform2D& visitState);

        /// @brief the path to the configuration file for WalkPathFollower
        static constexpr const char* CONFIGURATION_PATH = "WalkPathFollower.yaml";

        WalkPath currentPath;

    private:
        /// @brief Subsumption ID key to access motors
        const size_t subsumptionId;

        /// @brief Reaction handle for the path following reaction
        ReactionHandle followPathReaction;
        
        /// @brief Reaction handle for the path update reaction
        ReactionHandle updatePathReaction;


        struct Config {
            
            double waypoint_visit_distance = 0.1;
            bool draw_estimated_path = false;
            double walk_about_x_strafe = 0;
            double walk_about_y_strafe = 0;
            double walk_about_rotational_speed = 0;

        } cfg_;
    };

}
}
}

#endif  // MODULES_BEHAVIOUR_SKILLS_WALKPATHFOLLOWER_H
