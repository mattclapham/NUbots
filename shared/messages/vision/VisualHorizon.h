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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_VISION_VISUALHORIZON_H
#define MESSAGES_VISION_VISUALHORIZON_H

#include <armadillo>

#include "messages/input/Image.h"
#include "messages/vision/LookUpTable.h"

namespace messages {
    namespace vision {

        template <int camID>
        struct VisualHorizon {

            // The image that the horizon is from
            std::shared_ptr<const input::Image<camID>> image;
            std::shared_ptr<const LookUpTable> lut;

            // A colleciton of normals to the horizon planes, A vector is above the horizon
            // if a multiplication with this matrix results in only positive values
            arma::mat horizon;

        };
    }
}

#endif // MESSAGES_VISION_VISIONOBJECTS_H
