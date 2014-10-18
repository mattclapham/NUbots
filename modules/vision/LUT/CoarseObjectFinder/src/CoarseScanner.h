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

#ifndef MODULES_VISION_VISUALHORIZON_H
#define MODULES_VISION_VISUALHORIZON_H

#include <nuclear>
#include <armadillo>

#include "messages/input/Image.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/LookUpTable.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace vision {

        /**
         * Classifies a raw image, producing the colour segments for object detection
         *
         * @author Trent Houliston
         */
        class CoarseScanner {
        private:
            arma::mat horizonScanPoints;

            double MIN_SIZE_PIXELS = 3.0; //this is the radians between scanlines
            double CAMERA_HEIGHT = 1.2; //this is the radians buffer above the horizon
            double MIN_POST_WIDTH = 10*2.54/100.0;
            double MIN_POST_HEIGHT = 39*2.54/100.0;
            double MIN_GROUNDOBJ_SIZE = 0.3;

            //create a convex hull from the outline polygon, giving us info about obstacles
            arma::mat generateAboveHorizonRays(const messages::input::Image& image) const;

            //scan the horizon to find the horizon outline polygon
            arma::mat generateBelowHorizonRays(const messages::input::Image& image) const;

        public:
            //find the IMU horizon, visual horizon and convex hull of the visual horizon
            std::map<uint,std::vector<arma::ivec2>> findObjects(const messages::input::Image& image,
                             const messages::vision::LookUpTable& lut,
                             const arma::mat& horizonNormals) const;
        };

    }  // vision
}  // modules

#endif  // MODULES_VISION_QUEXLUTCLASSIFIER_H

