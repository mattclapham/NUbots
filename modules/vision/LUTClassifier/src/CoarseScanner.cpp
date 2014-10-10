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
#include "QuexClassifier.h"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::vision::LookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        
        CoarseScanner(Config& config) {
            
                
            //save the values we need to generate ray vector lists
            for (const auto& fov config["fovs"]) {
                //XXX: store these somehow
                generateScanRays(fov["x"].as<double>(),fov["y"].as<double>());
            }
            
        }
        
        arma::mat CoarseScanner::generateScanRays(const double& x, const double& y, const bool rectilinear = true) const {
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
        
        //do a coarse scan for objects
        void CoarseScanner::findObjects(const messages::input::Image& image,
                               const messages::vision::LookUpTable& lut, 
                               const arma::mat& horizonNormals,
                               quex) {
            
            arma::mat33 camTransform = image.IMU.span(0,0,2,2);
            
            //make a rotation matrix that doesn't change x/y axes somehow!
            arma::mat33 ScanRayTransform;
            arma::vec2 rotvals = arma::normalise(camTransform.col(0).rows(0,1));
            ScanRayTransform << rotvals[0] << rotvals[1] << 0 << arma::endr
                             << -rotvals[1] << rotvals[0] << 0 << arma::endr
                             << 0 << 0 << 1;
            
            arma::mat alignedTransform = arma::dot(camTransform, ScanRayTransform);
            
            //get scanRays for the correct FOV
            //XXX: cache these eventually
            //also rotate the scanrays so the horizon is in the right place
            arma::mat scanRays = arma::dot(generateScanRays(image.FOV[0],image.FOV[1],image.lens.rectilinear), alignedTransform);
            
            //trim the scanrays using the visual horizon
            if (horizonNormals.n_elem > 0) {
                scanRays = scanRays.rows(arma::find(arma::prod(arma::dot(scanRays,horizonNormals.t()) < 0, 1)));
            }
            
            //trim the scanrays to the field of view
            scanRays = trimToFOV(scanRays,image);
            
            
            //convert to pixels
            arma::imat pixels = bulkRay2Pixel(scanRays,image);
            
            
            //find all the unique pixels
            
            
            
            //do the LUT scans
            
            
            //emit
        }

    }  // vision
}  // modules
