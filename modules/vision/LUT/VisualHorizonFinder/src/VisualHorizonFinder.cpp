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

#include "VisualHorizonFinder.h"
#include "utility/vision/geometry/screen.h"
#include "messages/input/Image.h"

namespace modules {
namespace vision {
namespace LUT {
    
    using utility::vision::geometry::bulkRay2Pixel;
    using utility::vision::geometry::bulkPixel2Ray;
    using utility::vision::geometry::trimToFOV;
    using utility::vision::geometry::snapToScreen;
    using utility::vision::geometry::camTiltMatrix;
    using messages::input::Image;
    
    VisualHorizonFinder::VisualHorizonFinder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

    }


    arma::mat VisualHorizonFinder::generateScanRays(const double& x, const double& y, const bool rectilinear) const {
        //XXX: this currently assumes rectilinear or radial - fix should go in screen.h
        const double maxFOV = (rectilinear) ? sqrt(x*x + y*y) : std::max(x,y);
        arma::mat scanRays(uint(maxFOV/VISUAL_HORIZON_SCAN_RESOLUTION),3);


        //this is the rotation above the horizon to allow a buffer for detectign it
        const double sz = sin(VISUAL_HORIZON_BUFFER);
        const double cz = cos(VISUAL_HORIZON_BUFFER);

        //this calculates all the top camera raypoints in sphere space - we scan along the down vector from these
        for (uint i = 0; i < scanRays.n_rows; ++i) {
            const double sp = sin(i*VISUAL_HORIZON_SCAN_RESOLUTION - maxFOV/2);
            const double cp = cos(i*VISUAL_HORIZON_SCAN_RESOLUTION - maxFOV/2);
            scanRays.row(i) = arma::vec3({cp*cz,sp*cz,sz}).t();
        }

        return scanRays;
    }

    //find the IMU horizon, visual horizon and convex hull of the visual horizon
    arma::mat VisualHorizonFinder::findVisualHorizon(const messages::input::Image& image,
                           const messages::vision::LookUpTable& lut) {
        
        //initialize camera tilt matrix
        arma::mat33 camTransform = camTiltMatrix(image);

        //get scanRays for the correct FOV
        //XXX: cache these eventually
        arma::mat scanRays;
        if (image.lens.type == Image::Lens::Type::RADIAL) {
            scanRays = generateScanRays(image.lens.parameters.radial.radialFOV,image.lens.parameters.radial.radialFOV,false);
        } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
            scanRays = generateScanRays(image.lens.parameters.equirectangular.FOV[0],image.lens.parameters.equirectangular.FOV[1],true);
        }
        
        //trim out of screen pixels here
        arma::mat rayPositions = arma::round(
                                    bulkRay2Pixel(
                                        trimToFOV(
                                            camTransform*scanRays,
                                            image),
                                        image)
                                    );

        //get the down vector to project rays through
        arma::vec rayLength = -camTransform.col(2).rows(0,1);

        //shrink rays until all are the right length
        arma::mat rayEnds = snapToScreen(rayPositions,rayLength,image);

        //Then scan all rays
        auto scannedPts = quex->scanAll(arma::conv_to<arma::imat>::from(rayPositions), arma::conv_to<arma::imat>::from(rayEnds), lut);


        //then find the horizon points
        arma::imat horizonPts;

        //Remember: untransform to be in camera space
        arma::mat horizonRays = camTransform.t()*bulkPixel2Ray(horizonPts, image);

        arma::mat horizonNormals(horizonRays.n_rows,3);
        int startRay = 0;
        int endRay = 1;
        int totalNormals = 0;
        while (startRay < int(horizonRays.n_rows - 1)) {
            //initialise the starting hyperplane normal
            arma::vec currentNormal = arma::normalise(arma::cross(horizonRays.row(startRay), horizonRays.row(endRay)));
            
            //look for points above the hull
            arma::uvec aboveHull = arma::find(
                                        arma::vec(arma::dot(
                                            horizonRays.rows(
                                                startRay,
                                                horizonRays.n_rows-1),
                                            currentNormal))
                                        > 0.0);
            
            //keep taking the next value above the hull until we are convex
            while (endRay < int(horizonRays.n_rows - 1) and aboveHull.n_elem > 0.0) {
                endRay = aboveHull[0] + startRay;

                currentNormal = arma::normalise(arma::cross(horizonRays.row(startRay), horizonRays.row(endRay)));

                aboveHull = arma::find(
                                arma::vec(arma::dot(
                                    horizonRays.rows(
                                        startRay,
                                        horizonRays.n_rows-1),
                                    currentNormal))
                                > 0.0);
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

}
}
}

