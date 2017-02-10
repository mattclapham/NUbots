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

#ifndef MODULES_RESEARCH_AUTOCLASSIFIERPROVIDER_H
#define MODULES_RESEARCH_AUTOCLASSIFIERPROVIDER_H

#include <nuclear>
#include "message/vision/LookUpTable.h"
#include "message/vision/VisionObjects.h"

namespace module {
namespace research {

    class AutoClassifierProvider : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the AutoClassifier reactor.
        explicit AutoClassifierProvider(std::unique_ptr<NUClear::Environment> environment);

    private:
        int ballEdgeBuffer  = 0;
        uint8_t ballLightnessMin = 0;
        uint8_t ballLightnessMax = 255;
        int goalEdgeBuffer  = 0;
        uint8_t goalLightnessMin = 0;
        uint8_t goalLightnessMax = 255;
        int fieldEdgeBuffer = 0;
        uint8_t fieldLightnessMin = 0;
        uint8_t fieldLightnessMax = 255;
        int lineEdgeBuffer  = 0;
        int lineLightnessMin = 0;
        int lineLightnessMax = 255;

        void handleVisionGoals(const message::vision::VisionObject& goals);
        void handleVisionBalls(const message::vision::VisionObject& balls);

        ReactionHandle ballProvider;
        ReactionHandle goalProvider;
        ReactionHandle fieldProvider;
        ReactionHandle lineProvider;
    };

}
}


#endif
