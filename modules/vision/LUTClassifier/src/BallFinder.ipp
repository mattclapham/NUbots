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

#include "LUTClassifier.h"
#include "QuexClassifier.h"

#include "utility/math/geometry/Line.h"
#include "utility/vision/geometry/sphere.h"
#include "utility/vision/geometry/screen.h"
#include "utility/math/vision.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::ServoID;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;

        using utility::math::geometry::Line;
        using utility::math::vision::getGroundPointFromScreen;
        using utility::math::vision::projectWorldPointToScreen;
        using utility::math::vision::screenToImage;
        using utility::math::vision::imageToScreen;
        using utility::vision::geometry::sphere::arcSizeFromBaseRay;
        using utility::vision::geometry::bulkPixel2Ray;

        template <int camID>
        void LUTClassifier::findBall(const Image<camID>& image, const LookUpTable& lut, ClassifiedImage<ObjectClass, camID>& classifiedImage) {

            /*
                Here we cast lines to find balls.
                To do this, we cast lines seperated so that any ball will have at least 2 lines
                passing though it (possibly 3).
                This means that lines get logrithmically less dense as we decend the image as a balls
                apparent size will be larger.
                These lines are cast from slightly above the visual horizon to a point where it is needed
                (for the logrithmic grid)
             */
            
            double radius = image.lens.parameters.radial.fov/2/image.lens.parameters.radial.pitch;
            
            
            int dx = 3;
            
            
            for(double y = 0; y <= 400; y += dx) {
                
                
                double xSize = std::sqrt(radius*radius - y*y);
                int xStart = int(image.lens.parameters.radial.centre[0] - radius);
                int xEnd = int(image.lens.parameters.radial.centre[0] + radius);
                
                
                auto segments = quex->classify(image, lut, arma::ivec2({xStart,int(y+image.lens.parameters.radial.centre[1])}),
                                                           arma::ivec2({xEnd,int(y+image.lens.parameters.radial.centre[1])}), dx);
                
                
                insertSegments(classifiedImage, segments, false);
                
                
                arma::mat ray = bulkPixel2Ray(arma::ivec2({0, int(y+image.lens.parameters.radial.centre[1])}).t(), image );
                double arcSize = arcSizeFromBaseRay(ray.t(), 
                                                    0.4826, 
                                                    1.2 )[0];
                dx = std::max(
                        int(arcSize / 2.0 /image.lens.parameters.radial.pitch),
                        3);
                
            }

        }

    }  // vision
}  // modules
