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
                                                const double radialSize,
                                                const messages::input::Image& image) const {
            arma::running_stat_vec<arma::ivec2> stats;
            
            for (uint i = 0; i < selectedpts.n_rows; ++i) {
                stats.stat(allpts[selectedpts[i]]);
            }
            
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
                pointSizes[c] = vision::geometry::spheredist(something);
                
                /*pointSizes[c] = arma::vec(colourRays[c].n_rows);
                for (uint i = 0; i < colourRays[c].n_rows; ++i) {
                    pointSizes[c][i] = vision::geometry::spheredist(something);
                }*/
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
                arma::ivec active(colourRays[c].size(),1);
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
