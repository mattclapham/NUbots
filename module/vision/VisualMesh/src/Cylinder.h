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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_VISION_CYLINDER_H
#define MODULE_VISION_CYLINDER_H

#include <cmath>
#include <limits>

#include "message/vision/MeshObjectRequest.h"
#include "Circle.h"

namespace module {
namespace vision {

    class Cylinder {
    public:

        static inline double deltaPhi(const message::vision::MeshObjectRequest& request, double phi, double cameraHeight) {

            // Look at spheres at the top and base and choose the smallest one
            auto topRequest = request;
            double top = Circle::deltaPhi(topRequest, phi, cameraHeight);

            auto baseRequest = request;
            baseRequest.height = 0;
            auto base = Circle::deltaPhi(baseRequest, phi, cameraHeight);

            return std::min(top, base);
        }

        static inline double deltaTheta(const message::vision::MeshObjectRequest& request, double phi, double cameraHeight) {

            // Look at spheres at the top and base and choose the smallest one
            auto topRequest = request;
            double top = Circle::deltaTheta(topRequest, phi, cameraHeight);

            auto baseRequest = request;
            baseRequest.height = 0;
            auto base =  Circle::deltaTheta(baseRequest, phi, cameraHeight);

            return std::min(top, base);
        }
    };

}
}

#endif  // MODULE_VISION_CYLINDER_H
