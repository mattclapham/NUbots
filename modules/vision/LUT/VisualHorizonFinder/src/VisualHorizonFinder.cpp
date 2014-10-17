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

// quex lexer settings
#define QUEX_SETTING_BUFFER_MIN_FALLBACK_N 0
#define QUEX_OPTION_ASSERTS_DISABLED
#define QUEX_OPTION_COMPUTED_GOTOS
#define QUEX_OPTION_TERMINATION_ZERO_DISABLED

#include "VisualHorizonLexer.hpp"

#include "messages/input/Image.h"
#include "utility/vision/QuexClassifier.h"
#include "utility/vision/geometry/screen.h"

namespace modules {
namespace vision {
namespace LUT {

    using utility::vision::geometry::bulkRay2Pixel;
    using utility::vision::geometry::bulkPixel2Ray;
    using utility::vision::geometry::trimToFOV;
    using utility::vision::geometry::snapToScreen;
    using utility::vision::geometry::camTiltMatrix;
    using messages::input::Image;
    using messages::vision::LookUpTable;

    VisualHorizonFinder::VisualHorizonFinder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Image>, With<LookUpTable>>("Visual Horizon", [this](const Image& image, const LookUpTable& lut) {

            findVisualHorizon(image, lut);
        });

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
    arma::mat VisualHorizonFinder::findVisualHorizon(const Image& image,
                           const LookUpTable& lut) {

        //initialize camera tilt matrix
        arma::mat33 camTransform = camTiltMatrix(image);

        //get scanRays for the correct FOV
        //XXX: cache these eventually
        arma::mat scanRays;
        if (image.lens.type == Image::Lens::Type::RADIAL) {
            scanRays = generateScanRays(image.lens.parameters.radial.fov, image.lens.parameters.radial.fov, false);
        } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
            scanRays = generateScanRays(image.lens.parameters.equirectangular.fov[0],image.lens.parameters.equirectangular.fov[1],true);
        }
        
        
        //trim out of screen pixels here
        arma::mat rayPositions = bulkRay2Pixel(
                                        trimToFOV(
                                            (scanRays*camTransform),
                                            image),
                                        image);
        
        //get the down vector to project rays through
        arma::vec rayLength = -camTransform.submat(0,2,1,2);//.col(2).rows(0,1).t();
        //shrink rays until all are the right length
        arma::mat rayEnds = snapToScreen(rayPositions,rayLength,image);
        
        arma::imat starts = arma::conv_to<arma::imat>::from(rayPositions.t());
        arma::imat ends = arma::conv_to<arma::imat>::from(rayEnds);
        
        //initialize the horizon points
        arma::imat horizonPts(starts.n_cols,2);
        
        
        // Scan all our segments
        for(uint i = 0; i < starts.n_cols; ++i) {
            
            
            const arma::ivec2 s = starts.col(i);
            
            const arma::ivec2 e = ends.col(i);
            
            auto pts = utility::vision::bresenhamLine(s, e);
            
            std::vector<char> l;
            l.reserve(pts.size());

            // Lut all the points
            for(auto& p : pts) {
                l.push_back(lut(image(p[0], p[1])));
            }

            auto segments = utility::vision::quexClassify<quex::VisualHorizonLexer>(l.begin(), l.end());
            
            
            //save the top VH segment
            int cnt = 0;
            for (auto& s : segments) {
                if (s.first == QUEX_TKN_FIELD and s.second > VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE) {
                    horizonPts.row(i) = pts[cnt].t();
                    cnt = -1;
                    break;
                }
                cnt += s.second;
            }
            
            //if we find no VH segments
            if (cnt >= 0) {
                horizonPts.row(i) = pts.back().t();
            }
        }

        
        
        //Remember: untransform to be in camera space
        arma::mat horizonRays = (bulkPixel2Ray(horizonPts, image)*camTransform);
        arma::mat horizonNormals(horizonRays.n_rows,3);
        int startRay = 0;
        int endRay = 1;
        int totalNormals = 0;
        
        /*
        std::cout << starts.t() << std::endl << ends.t() << std::endl << horizonPts << std::endl;
        
        arma::mat m1 = arma::conv_to<arma::mat>::from(horizonPts);
        arma::mat m2 = bulkRay2Pixel(bulkPixel2Ray(horizonPts, image),image);
        std::cout << m1 - m2;
        std::cout << std::endl;
        std::cout << m1.rows(26,29) << std::endl;
        std::cout << m2.rows(26,29) << std::endl;
        */
        
        while (startRay < int(horizonRays.n_rows - 1)) {
            //initialise the starting hyperplane normal
            arma::vec currentNormal = -arma::cross(horizonRays.row(startRay).t(), horizonRays.row(endRay).t());
            
            
            //look for points above the hull
            if (endRay != horizonRays.n_rows-1)  {
                arma::uvec aboveHull = arma::find(
                                    arma::prod(
                                        horizonRays.rows(endRay+1, horizonRays.n_rows-1) *
                                        currentNormal >= 0.0, 1));
                //keep taking the next value above the hull until we are convex
                while (endRay < int(horizonRays.n_rows - 1) and aboveHull.n_elem > 0) {
                    endRay = aboveHull[0] + endRay + 1;
                    currentNormal = -arma::cross(horizonRays.row(startRay).t(), horizonRays.row(endRay).t());
                    if (endRay !=  int(horizonRays.n_rows-1)) {
                        aboveHull = arma::find(
                                        arma::prod(
                                            horizonRays.rows(endRay+1, horizonRays.n_rows-1) *
                                            currentNormal >= 0.0, 1));
                    } else {
                        aboveHull = arma::uvec();
                    }
                }
                if (aboveHull.n_elem > 0) {
                    endRay = aboveHull[0] + endRay + 1;
                    currentNormal = -arma::cross(horizonRays.row(startRay).t(), horizonRays.row(endRay).t());
                }
                horizonNormals.row(totalNormals) = currentNormal.t();
                ++totalNormals;
            } else {
                /*std::cout  << "Final violations: " << arma::find(
                                    arma::prod(
                                        horizonRays.rows(0, horizonRays.n_rows-1) *
                                        currentNormal < 0.0, 1)).t();*/
            }
            
            startRay = endRay;
            ++endRay;
        }
        if (totalNormals < horizonNormals.n_rows) {
            horizonNormals.shed_rows(totalNormals,horizonNormals.n_rows-1);
        }
        
        std::cout  << std::endl << "Normals:" << std::endl << horizonNormals << std::endl;
        //return the hyperhull
        return std::move(horizonNormals);
    }

}
}
}

