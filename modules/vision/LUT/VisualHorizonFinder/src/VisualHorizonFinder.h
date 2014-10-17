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

#ifndef MODULES_VISION_LUT_VISUALHORIZONFINDER_H
#define MODULES_VISION_LUT_VISUALHORIZONFINDER_H

#include <nuclear>
#include <armadillo>

#include "messages/vision/LookUpTable.h"
#include "messages/input/Image.h"

namespace modules {
namespace vision {
namespace LUT {

    class VisualHorizonFinder : public NUClear::Reactor {
    private:
        double VISUAL_HORIZON_SCAN_RESOLUTION = 0.1; //this is the radians between scanlines
        double VISUAL_HORIZON_BUFFER = 0.1; //this is the radians buffer above the horizon
        double VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE = 0;
        double VISUAL_HORIZON_SUBSAMPLING = 1;

        arma::mat generateScanRays(const double& x, const double& y, const bool rectilinear = true) const;

        //find the IMU horizon, visual horizon and convex hull of the visual horizon
        arma::mat findVisualHorizon(const messages::input::Image& image,
                               const messages::vision::LookUpTable& lut);
    public:
        /// @brief Called by the powerplant to build and setup the VisualHorizonFinder reactor.
        explicit VisualHorizonFinder(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}


#endif
