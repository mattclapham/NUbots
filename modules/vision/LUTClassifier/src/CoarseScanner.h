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

#include "messages/input/Image.h"
#include "messages/input/Sensors.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/LookUpTable.h"

namespace modules {
    namespace vision {

        struct LUTLocation {
            static constexpr const char* CONFIGURATION_PATH = "LookUpTable.yaml";
        };

        class QuexClassifier;

        /**
         * Classifies a raw image, producing the colour segments for object detection
         *
         * @author Trent Houliston
         */
        class VisualHorizonFinder {
        private:
            arma::mat horizonScanPoints;
            
            // A pointer to our quex class (since it is generated it is not defined at this point)
            QuexClassifier* quex;

            int VISUAL_HORIZON_SCAN_RESOLUTION = 0.2; //this is the radians between scanlines
            double VISUAL_HORIZON_BUFFER = 0.1; //this is the radians buffer above the horizon
            double VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE = 0;
            double VISUAL_HORIZON_SUBSAMPLING = 1;
            
            //create a convex hull from the outline polygon, giving us info about obstacles
            void findVisualHull(const messages::input::Image& image,
                                const messages::vision::LookUpTable& lut, 
                                messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);
            
            //scan the horizon to find the horizon outline polygon
            void scanHorizon(const messages::input::Image& image,
                             const messages::vision::LookUpTable& lut,
                             const arma::mat44& cameraToGround,
                             messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);
            
            //find the IMU horizon, visual horizon and convex hull of the visual horizon
            void findVisualHorizon(const messages::input::Image& image,
                                   const messages::vision::LookUpTable& lut, 
                                   const arma::mat44& cameraToGround,
                                   messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage);

        public:
            //static constexpr const char* CONFIGURATION_PATH = "LUTClassifier.yaml";

            VisualHorizonFinder(config);
        };

    }  // vision
}  // modules

#endif  // MODULES_VISION_QUEXLUTCLASSIFIER_H

