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

#include "CoarseScanner.h"
#include <cmath>
#include "utility/vision/geometry/screen.h"
#include "utility/vision/geometry/sphere.h"
#include "utility/vision/geometry/cylinder.h"



namespace modules {
    namespace vision {

        using messages::input::Image;
        using utility::vision::geometry::camTiltMatrix;
        using utility::vision::geometry::bulkRay2Pixel;
        using utility::vision::geometry::bulkPixel2Ray;
        using utility::vision::geometry::trimToFOV;
        
        /*CoarseScanner(Config& config) {
            
                
            //save the values we need to generate ray vector lists
            for (const auto& fov config["fovs"]) {
                //XXX: store these somehow
                generateScanRays(fov["x"].as<double>(),fov["y"].as<double>());
            }
            
        }*/
        
        /*arma::mat CoarseScanner::generateScanRays(const double& x, const double& y, const bool rectilinear = true) const {
            //XXX: this currently assumes rectilinear - radial would be max of x and y
            const double maxFOV = (rectilinear) ? sqrt(x*x + y*y) : std::max(x,y);
            arma::mat scanRays(uint(maxFOV/VISUAL_HORIZON_SCAN_RESOLUTION),3);
            
            uint total = 0;
            
            //this is the rotation above the horizon to allow a buffer for detectign it
            double sz = sin(VISUAL_HORIZON_BUFFER);
            double cz = cos(VISUAL_HORIZON_BUFFER);
            
            //this calculates all the top camera raypoints in sphere space - we scan along the down vector from these
            for (uint i = 0; i < scanrays.n_rows; ++i) {
                sp = sin(i*VISUAL_HORIZON_SCAN_RESOLUTION - maxFOV/2);
                cp = cos(i*VISUAL_HORIZON_SCAN_RESOLUTION - maxFOV/2);
                scanRays[i] = arma::rowvec({cp*cz,sp*cz,sz});
            }
            
            return scanRays;
        }*/
        arma::mat CoarseScanner::generateAboveHorizonRays(const Image& image) {
            
            //get the max possible FOV, and the estimated pixel size
            double maxFOV;
            double pixelSize;
            if (image.lens.type == Image::Lens::Type::RADIAL) {
                maxFOV  = image.lens.parameters.radial.radialFOV;
                pixelSize = image.lens.parameters.radial.pixelPitch;
            } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
                maxFOV = arma::norm(arma::vec2({image.lens.parameters.equirectangular.FOV[0], image.lens.parameters.equirectangular.FOV[1]}));
                pixelSize = maxFOV/arma::norm(arma::vec2({double(image.dimensions[0]), double(image.dimensions[1])}));
            }
            
            //work out our limits
            //XXX: this might need to be replaced by something simpler
            double angleLimit = std::min(maxFOV, M_PI);
            angleLimit = std::min(angleLimit, 
                         utility::vision::geometry::cylinder::arcSizeFromTopRayVertical(
                                    arma::vec3({cos(M_PI/2),0.0,sin(M_PI/2)}), 
                                    arma::vec2({MIN_POST_WIDTH,MIN_POST_HEIGHT}), 
                                    CAMERA_HEIGHT)[1]);
            
            //create our ray matrix
            arma::mat scanRays(0,0);
            
            //define the starting angle to scan from
            double startAngle = 0.0 + pixelSize*MIN_SIZE_PIXELS/2;
            double offset = 0.0;
            arma::vec2 halfArcSize = arma::vec2({0.0,0.0});
            
            //loop through creating new rays
            //XXX: for loops everywhere, to make Trent proud
            for (startAngle = 0.0 + pixelSize*MIN_SIZE_PIXELS/2; startAngle < angleLimit; startAngle += halfArcSize[1]) {
                arma::vec3 camRay = arma::vec3({cos(startAngle),0.0,sin(startAngle)});
                halfArcSize = utility::vision::geometry::cylinder::arcSizeFromBaseRayVertical(
                                    arma::vec3({cos(M_PI/2.0),0.0,sin(M_PI/2.0)}), 
                                    arma::vec2({MIN_POST_WIDTH,MIN_POST_HEIGHT}),
                                    CAMERA_HEIGHT)/2.0;
                //scale because we're mapping in spherical coordinates here
                double scaledArcWidth = halfArcSize[0];
                
                
                int numRays = int((maxFOV - offset)/scaledArcWidth);
                scanRays = arma::resize(scanRays, scanRays.n_rows + numRays, 3);
                
                double cosSA = cos(startAngle);
                double sinSA = sin(startAngle);
                
                for (int i = 0; i < numRays; ++i) {
                    scanRays.row(scanRays.n_rows - i - 1) = arma::vec3({ cos(double(i)*scaledArcWidth - maxFOV/2.0 + offset) * cosSA, 
                                                                         sin(double(i)*scaledArcWidth - maxFOV/2.0 + offset) * cosSA, 
                                                                         sinSA}).t();
                }
                
                startAngle += halfArcSize[1];
            }
            
            return scanRays;
        }
        
        arma::mat CoarseScanner::generateBelowHorizonRays(const Image& image) {
            return arma::mat();
        }
        
        //do a coarse scan for objects
        void CoarseScanner::findObjects(const messages::input::Image& image,
                               const messages::vision::LookUpTable& lut, 
                               const arma::mat& horizonNormals) {
            //world space
            arma::mat33 camTransform = camTiltMatrix(image);
            
            //x = forward tilted cam space
            arma::mat alignedTransform = camTiltMatrix(image);
            
            //get scanRays for the correct FOV
            //XXX: cache these eventually
            arma::mat aboveHorizonRays = camTransform * generateAboveHorizonRays(image);
            arma::mat belowHorizonRays = camTransform * generateBelowHorizonRays(image);
            /*if (image.lens.type == Image::Lens::Type::RADIAL) {
            //old code
            } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
                aboveHorizonRays = camTransform * generateAboveHorizonRays(image);
                belowHorizonRays = camTransform * generateBelowHorizonRays(image.lens.parameters.equirectangular.FOV[0],
                                                                           image.lens.parameters.equirectangular.FOV[1],
                                                                           true);
            }*/
            
            //trim the scanrays using the visual horizon
            if (horizonNormals.n_elem > 0) {
                belowHorizonRays = belowHorizonRays.rows(arma::find(arma::prod(belowHorizonRays * horizonNormals.t() < 0.0, 1)));
            }
            
            //trim the scanrays to the field of view
            aboveHorizonRays = trimToFOV(aboveHorizonRays,image);
            belowHorizonRays = trimToFOV(belowHorizonRays,image);
            
            
            //convert to pixels
            arma::imat aboveHorizonPixels = arma::conv_to<arma::imat>::from(bulkRay2Pixel(aboveHorizonRays,image));
            arma::imat belowHorizonPixels = arma::conv_to<arma::imat>::from(bulkRay2Pixel(belowHorizonRays,image));
            
            
            //find all the unique pixels
            
            
            
            //do the LUT scans
            
            
            //emit
        }

    }  // vision
}  // modules
