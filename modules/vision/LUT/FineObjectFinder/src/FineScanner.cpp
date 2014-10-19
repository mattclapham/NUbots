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

#include "FineScanner.h"
#include <cmath>
#include <map>
#include <set>
#include <vector>
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
        
        arma::mat FineScanner::fineScanClassify(const std::vector<arma::ivec>& allpts, 
                                                const arma::uvec& selectedpts, 
                                                const messages::input::Image& image) const {
            
            //collect data on our points
            arma::running_stat_vec<arma::ivec2> stats;
            for (uint i = 0; i < selectedpts.n_rows; ++i) {
                stats.stat(allpts[selectedpts[i]]);
            }
            
            double radialDiameterPx = vision::geometry::arcSizeFromBaseRay(
                                    bulkPixel2Ray(arma::conv_to<vec2>::from(stats.min()).t(),image),
                                    MAX_OBJECT_SIZE,CAMERA_HEIGHT));
            
            if (image.lens.type == Image::Lens::Type::RADIAL) {
                radialDiameterPx *= image.lens.parameters.radial.pitch;
            } else if (image.lens.type == Image::Lens::Type::EQUIRECTANGULAR) {
                //XXX: zen hack
                pitch = sqrt(image.lens.parameters.equirectangular.fov[0]*image.lens.parameters.equirectangular.fov[0]
                             image.lens.parameters.equirectangular.fov[1]*image.lens.parameters.equirectangular.fov[1]) / 
                             sqrt(image.dimensions[0]*image.dimensions[0] + image.dimension[1]*image.dimensions[1]);
                radialDiameterPx *= pitch;
            }
            
            arma::ivec2 center = (stats.max() + stats.min());
            //find the diameter of the object
            int diameter = int(std::min<double>(arma::norm(arma::conv_to<arma::vec2>::from(stats.max() - stats.min())), 
                                radialDiameterPx) +
                                MIN_SURROUNDING_PIXELS + 0.5);
            
            //make horizontal lines
            int minPixelY = std::max(center[1]-diameter,0);
            int maxPixelY = std::min(center[1]+diameter,image.dimensions[1]);
            int increment = std::max(diameter/CROSSHATCH_LINES,1);
            
            for (int i = minPixelY; i < maxPixelY; i += increment) {
                int diff = int(sqrt(diameter - i*i));
                int leftX = std::max(centre[0]-diff, 0);
                int rightX = std::min(centre[0]+diff, image.dimensions[0]);
            }
            
            //XXX: make vertical lines
            
            //XXX: return lines
            return arma::mat();
        }
        
        std::map<uint,std::vector<arma::ivec2>> FineScanner::findObjects(const messages::input::Image& image,
                                                            const messages::vision::LookUpTable& lut,
                                                            const arma::mat& horizonNormals,
                                                            const std::map<uint,std::vector<arma::ivec2>>& coarseScan) const {
            
            //1. build matrices for each colour of interest (from the std::vecs of pixels in the input)
            //(1.5) - convert to rays
            std::map<uint,arma::mat> colourRays;
            for (const auto& c : candidateClours) {
                arma::imat tmp(coarseScan[c].size(),2);
                for (uint i = 0; i < coarseScan[c].size(); ++i) {
                    tmp.row(i) = coarseScan[c][i].t();
                }
                colourRays[c] = bulkPixel2Ray(tmp);
            }
            
            //2. query size estimates for each colour of interest (from vision::geometry)
            // (create a vec of these)
            std::map<uint,arma::vec> pointSizes;
            //XXX: rewrite distances to work with mats in bulk?
            for (const auto& c : candidateClours) {
                //with matrix based version
                pointSizes[c] = vision::geometry::arcSizeFromBaseRay(colourRays[c].t(),MAX_OBJECT_SIZE,CAMERA_HEIGHT);
            }
            
            //3. build cosine distances for each colour of interest ( a * b.t() )
            // (will be a square matrix)
            std::map<uint,arma::mat pointDists;
            for (const auto& c : candidateClours) {
                pointDists[c] = colourRays[c] * colourRays[c].t();
            }
            
            //4. clustering for each colour: 
            // (init a bool list for marked off rows)
            // - start at first row
            // - threshold dists based on size estimates ( arma::find(a.row(i) > thresh[i] )
            // - take all above threshold as a cluster in the first row (for cylinders, propagate these through and include neighbours of neighbours, etc)
            // - mark off clustered points
            // - proceed to the next unmarked row
            
            std::vector<arma::imat> fineScanLines;
            for (const auto& c : candidateClours) {
                arma::ivec active(colourRays[c].size());
                active = 1;
                for (uint i = 0; i < colourRays[c].n_rows; ++i) {
                    
                    if (active[i]) {
                        arma::uvec activePixels = arma::find(pointDists[c][i] <= std::max(pointSizes[c][i], MIN_ANGULAR_SIZE));
                        active(activePixels) = 0;
                        
                        fineScanLines.push_back( fineScan( coarseScan[c], activePixels , image ) );
                    }
                }
            }
            
            //5. aggregate scanlines
            
            
            //6. do scans
            
            
            //7. emit
            
            return {};
        }

    }  // vision
}  // modules
