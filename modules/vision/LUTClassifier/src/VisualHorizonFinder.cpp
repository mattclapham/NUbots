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

#include "VisualHorizonFinder.h"
#include "QuexClassifier.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        
        VisualHorizonFinder(Config& config) {
            
                
            //save the values we need to generate ray vector lists
            for (const auto& fov config["fovs"]) {
                //XXX: store these somehow
                generateScanRays(fov["x"].as<double>(),fov["y"].as<double>());
            }
            
        }
        
        arma::mat VisualHorizonFinder::generateScanRays(const double& x, const double& y, const bool rectilinear = true) const {
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
        }
        
        //find the IMU horizon, visual horizon and convex hull of the visual horizon
        void findVisualHorizon(const messages::input::Image& image,
                               const messages::vision::LookUpTable& lut, 
                               const arma::mat44& cameraToGround,
                               messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage) {
            
            //XXX: get scanRays for the correct FOV
            
            
            //trim out of screen pixels here
            arma::imat rayPositions = trimToScreen(
                                            bulkRay2Pixel(
                                                IMU*scanRays,
                                                image),
                                            image);
            
            arma::ivec rayLength = arma::conv_to<arma::ivec>::from(arma::round(IMU.col(2)*(imageSize)));
            
            //shrink rays until all are the right length
            //XXX: this will look different for radial and rectilinear
            arma::imat rayEnds = snapToScreen(rayPositions,rayLength,image);
            
            //Then scan all rays
            auto scannedPts = quex->scanAll(rayPositions,rayEnds);
            
            
            //then find the horizon points
            arma::imat horizonPts;
            
            //Remember: untransform to be in camera space
            arma::mat horizonRays = IMU.t()*bulkPixel2Ray(arma::conv_to<arma::mat>::from(horizonPts));
            
            
            //then find the spherical hyperhull
            
            
            //return the hyperhull
        }

    }  // vision
}  // modules
