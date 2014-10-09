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
                               messages::vision::ClassifiedImage<messages::vision::ObjectClass>& classifiedImage) {
            
            arma::mat33 camTransform = image.IMU.cols(0,2).rows(0,2);
            
            //get scanRays for the correct FOV
            //XXX: cache these eventually
            arma::mat scanRays = generateScanRays(image.FOV[0],image.FOV[1],image.lens.rectilinear);
            
            //trim out of screen pixels here
            arma::imat rayPositions = arma::conv_to<arma::imat>::from(mat)arma::round(
                                        trimToFOV(
                                            bulkRay2Pixel(
                                                camTransform*scanRays,
                                                image),
                                            image)));
            
            //get the down vector to project rays through
            arma::ivec rayLength = arma::round(-camTransform.col(2).rows(0,1));
            
            //shrink rays until all are the right length
            arma::imat rayEnds = snapToScreen(rayPositions,rayLength,image);
            
            //Then scan all rays
            auto scannedPts = quex->scanAll(rayPositions,rayEnds);
            
            
            //then find the horizon points
            arma::imat horizonPts;
            
            //Remember: untransform to be in camera space
            arma::mat horizonRays = camTransform.t()*bulkPixel2Ray(arma::conv_to<arma::mat>::from(horizonPts));
            
            
            //then find the spherical hyperhull
            
            //
            
            arma::mat horizonNormals(horizonRays.n_rows,3);
            int startRay = 0;
            int endRay = 1;
            int totalNormals = 0;
            while (startRay < horizonRays.n_rows - 1) {
                arma::vec currentNormal = arma::normalise(arma::cross(horizonRays.row(startRay), horizonRays.row(endRay)));
                
                arma::ivec aboveHull = arma::find(arma::dot(horizonRays.rows(startRay,horizonRays.n_rows-1),currentRay) > 0);
                
                while (endRay < horizonRays.n_rows - 1 and aboveHull.n_elem > 0) {
                    endRay = aboveHull[0] + startRay;
                    
                    currentNormal = arma::normalise(arma::cross(horizonRays.row(startRay), horizonRays.row(endRay)));
                
                    aboveHull = arma::find(horizonRays.rows(startRay,horizonRays.n_rows-1) > 0);
                }
                
                horizonNormals.row(totalNormals) = currentNormal.t();
                
                startRay = endRay;
                ++endRay;
                ++totalNormals;
            }
            
            horizonNormals.shed_rows(totalNormals,horizonNormals.n_rows-1);
            
            //return the hyperhull
            return std::move(horizonNormals);
        }

    }  // vision
}  // modules
