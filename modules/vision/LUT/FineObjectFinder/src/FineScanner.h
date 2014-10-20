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

#ifndef MODULES_VISION_FINESCANNER_H
#define MODULES_VISION_FINESCANNER_H

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
        class FineScanner {
        private:
            
            double MAX_OBJECT_SIZE = 0.58;
            //double MAX_POST_WIDTH = 39*2.56/100.0;
            double CAMERA_HEIGHT = 1.2;
            int MIN_SURROUNDING_PIXELS = 5;
            double MIN_ANGULAR_SIZE = 0.005;
            int CROSSHATCH_LINES = 10;
            

            //create a classification outline
            arma::imat fineScanClassify( const std::vector<arma::ivec2>& allpts, 
                                        const arma::uvec& selectedpts, 
                                        const messages::input::Image& image) const;

        public:
            //find the IMU horizon, visual horizon and convex hull of the visual horizon
            std::map<uint,std::vector<arma::ivec2>> findObjects(const messages::input::Image& image,
                             const messages::vision::LookUpTable& lut,
                             const arma::mat& horizonNormals,
                             const std::map<uint,std::vector<arma::ivec2>>& coarseScan) const;
        };

    }  // vision
}  // modules

#endif  // MODULES_VISION_QUEXLUTCLASSIFIER_H

