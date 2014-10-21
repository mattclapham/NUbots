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
// quex lexer settings
#define QUEX_SETTING_BUFFER_MIN_FALLBACK_N 0
#define QUEX_OPTION_ASSERTS_DISABLED
#define QUEX_OPTION_COMPUTED_GOTOS
#define QUEX_OPTION_TERMINATION_ZERO_DISABLED

#include "FineObjectLexer.hpp"

#include "utility/vision/geometry/screen.h"
#include "utility/vision/geometry/sphere.h"
#include "utility/vision/geometry/cylinder.h"
#include "utility/vision/QuexClassifier.h"



namespace modules {
    namespace vision {

        using messages::input::Image;
        using utility::vision::geometry::camTiltMatrix;
        using utility::vision::geometry::bulkRay2Pixel;
        using utility::vision::geometry::bulkPixel2Ray;
        using utility::vision::geometry::trimToFOV;
        
        template <int camID>
        void FineScanner::insertSegments(messages::vision::ImageSegmentScan<camID>& image, std::vector<std::pair<uint32_t,uint32_t>>& segments, const uint& line, bool vertical) {
            typename messages::vision::ImageSegmentScan<camID>::Segment* previous = nullptr;
            typename messages::vision::ImageSegmentScan<camID>::Segment* current = nullptr;
    
            auto& target = image;
            int total = 0;
            for (auto& s : segments) {

                // Move in the data
                
                current = &(target.insert(std::make_pair(s.first, messages::vision::ImageSegmentScan<camID>::Segment({ 
                                                                arma::vec2({ vertical ? line : total,
                                                                            !vertical ? line : total}), 
                                                                arma::vec2({ vertical ? line : total+s.second,
                                                                            !vertical ? line : total+s.second}) }) ))->second);
                total += s.second;
                // Link up the results
                current->previous = previous;
                if(previous) {
                    previous->next = current;
                }

                // Get ready for our next one
                previous = current;
            }
        }
        
        arma::mat convexHull(const arma::mat& pts) {
            //ASSUMING pts is sorted!!!
            arma::uvec result(pts.n_rows);
            uint total = 0;
            // Do a convex hull on the map points to build the horizon
            for(uint i = 0; i < pts.n_rows; ++i) {

                auto b = i + 1;
                auto c = i + 2;

                // Get the Z component of a cross product to check if it is concave
                bool concave = 0 <   (double(pts(i,0)) - double(pts(b,0))) * (double(pts(c,1)) - double(pts(b,1)))
                                   - (double(pts(i,1)) - double(pts(b,1))) * (double(pts(c,0)) - double(pts(b,0)));

                if(concave) {
                    result(total) = i;
                    ++total;
                }
            }
            
            for(uint i = pts.n_rows-1; i < pts.n_rows; --i) {

                auto b = i + 1;
                auto c = i + 2;

                // Get the Z component of a cross product to check if it is concave
                bool concave = 0 <   (double(pts(i,0)) - double(pts(b,0))) * (double(pts(c,1)) - double(pts(b,1)))
                                   - (double(pts(i,1)) - double(pts(b,1))) * (double(pts(c,0)) - double(pts(b,0)));

                if(concave) {
                    result(total) = i;
                    ++total;
                }
            }
            
            return pts.rows(result.rows(0,total));
        }
        
        
        

        template <int camID>
        messages::vision::ImageSegmentScan<camID> FineScanner::findObjects(const messages::input::Image<camID>& image,
                                                            const messages::vision::LookUpTable& lut,
                                                            const arma::mat& horizonNormals,
                                                            const std::map<uint,std::vector<arma::ivec2>>& coarseScan) const {
            //XXX: implement dynamic candidates
            std::vector<uint> candidateColours;

            //get the pixel pitches
            double pitch;
            if (image.lens.type == Image<camID>::Lens::Type::RADIAL) {
                pitch = image.lens.parameters.radial.pitch;
            } else if (image.lens.type == Image<camID>::Lens::Type::EQUIRECTANGULAR) {
                //XXX: zen hack
                pitch = sqrt(image.lens.parameters.equirectangular.fov[0]*image.lens.parameters.equirectangular.fov[0] +
                             image.lens.parameters.equirectangular.fov[1]*image.lens.parameters.equirectangular.fov[1]) /
                             sqrt(image.dimensions[0]*image.dimensions[0] + image.dimensions[1]*image.dimensions[1]);
            }

            //1. build matrices for each colour of interest (from the std::vecs of pixels in the input)
            //(1.5) - convert to rays
            std::map<uint,arma::mat> colourRays;
            for (const uint& c : candidateColours) {
                arma::imat tmp(coarseScan.at(c).size(),2);
                for (uint i = 0; i < coarseScan.at(c).size(); ++i) {
                    tmp.row(i) = coarseScan.at(c)[i].t();
                }
                colourRays[c] = bulkPixel2Ray(tmp,image);
            }

            //2. query size estimates for each colour of interest (from vision::geometry)
            // (create a vec of these)
            std::map<uint,arma::vec> pointSizes;
            for (const auto& c : candidateColours) {
                //with matrix based version
                pointSizes[c] = utility::vision::geometry::sphere::arcSizeFromBaseRay(colourRays[c].t(),MAX_OBJECT_SIZE,CAMERA_HEIGHT);
            }

            //3. build cosine distances for each colour of interest ( a * b.t() )
            // (will be a square matrix)
            std::map<uint,arma::mat> pointDists;
            for (const auto& c : candidateColours) {
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
            for (const auto& c : candidateColours) {
                arma::ivec active(colourRays[c].size());
                active.fill(1);
                for (uint i = 0; i < colourRays[c].n_rows; ++i) {

                    if (active[i]) {
                        
                        
                        std::set<uint> candidates;
                        //keep expanding until we can't anymore
                        for (bool expanding = false; expanding; expanding = false) {
                            //go through all the current candidates
                            for (const auto& k : candidates) {
                                if (active[k]) {
                                    //if we haven't used it yet, add all neighbours
                                    arma::uvec activePixels = arma::find(pointDists[c].col(k) <= std::max(pointSizes[c][k], MIN_ANGULAR_SIZE));
                                    for (uint j = 0; j < activePixels.n_rows; ++j) {
                                        //if we include anything new, set expanded
                                        if (active[activePixels[j]]) {
                                            const uint coordKey = arma::accu( coarseScan.at(c)[k].t() % arma::ivec2({1,image.width()}) );
                                            candidates.insert(coordKey);
                                            expanding = true;
                                        }
                                    }
                                    active[activePixels[k]] = 0;
                                }
                                
                            }
                        }
                        
                        //make a list of points
                        arma::mat candidatePos(active.size(),2);
                        uint cnt = 0;
                        for (const auto& p : active) {
                            candidatePos[cnt] = arma::rowvec({ double(p % image.width()), double( p / image.width()) });
                            ++cnt;
                        }
                        
                        
                        //create a convex hull
                        candidatePos = convexHull(arma::sort(candidatePos));
                        
                        //expand the hull
                        arma::mat finalPts(candidatePos.n_rows,candidatePos.n_cols);
                        for (uint i = 0; i < candidatePos.n_rows; ++i) {
                            arma::vec pt1 = (candidatePos.row((i + 1)%candidatePos.n_rows) - candidatePos.row(i)).t();
                            arma::vec pt2 = (candidatePos.row((i + 1)%candidatePos.n_rows) - candidatePos.row((i + 2)%candidatePos.n_rows)).t();
                            
                            //XXX: the pointSizes lookup is the wrong one for this particular point, but I don't care right now
                            //XXX: doesn't handle parallel coincident lines
                            finalPts.row((i + 1)%candidatePos.n_rows) = candidatePos.row((i + 1)%candidatePos.n_rows) + 
                                                                        arma::normalise(pt1 + pt2).t() * pointSizes[c][i]/2;
                        }
                        
                        //push back the polygon
                        fineScanLines.push_back( arma::conv_to<arma::imat>::from(arma::round(finalPts)) );
                    }
                }
            }


            //6. do scans
            messages::vision::ImageSegmentScan<camID> result;
            for (const auto& f: fineScanLines) {
                //vertical scans
                const uint SCAN_DENSITY = 2;
                arma::uvec scanOrder = arma::sort_index(f.col(0));
                uint numScans = (f(scanOrder[scanOrder.n_rows-1],0) - f(scanOrder[0],0)) / SCAN_DENSITY;
                
                //these track the current segments
                uint left = scanOrder[0];
                uint right = scanOrder[0];
                uint leftNext = (left+1) % f.n_rows;
                uint rightNext = (right-1) % f.n_rows;
                double leftDiv = 1.0/(f( leftNext, 0) - f( left, 0));
                double leftLength = f( leftNext, 1) - f( left, 1);
                double rightDiv = 1.0/(f( rightNext, 0) - f( right, 0));
                double rightLength = f( rightNext, 1) - f( right, 1);
                
                for ( uint i = f(scanOrder[0],0); i <= f(scanOrder[scanOrder.n_rows-1],0); i += SCAN_DENSITY) {
                    //check if we've passed this segment
                    
                    if ( f( leftNext, 0 ) < i ) {
                     left = leftNext;
                     leftNext = (left+1) % f.n_rows;
                     leftDiv = 1.0/(f( leftNext, 0) - f( left, 0));
                     leftLength = f( leftNext, 1) - f( left, 1);
                    }
                    //calculate where we are on the line
                    int left = int(leftLength * (i - f( left, 0)) / leftDiv + f( left, 1) + 0.5);
                    
                    //check if we've passed this segment
                    if ( f( rightNext, 0 ) < i ) {
                     right = rightNext;
                     rightNext = (right-1) % f.n_rows;
                     rightDiv = 1.0/(f( rightNext, 0) - f( right, 0));
                     rightLength = f( rightNext, 1) - f( right, 1);
                    }
                    //calculate where we are on the line
                    int right = int(rightLength * (i - f( right, 0)) / rightDiv + f( right, 1) + 0.5);
                    
                    
                    //do quex from (i,left) to (i,right)
                    auto pts = utility::vision::bresenhamLine(arma::ivec2({i,left}), arma::ivec2({i,right}));
                    std::vector<char> l;
                    l.reserve(pts.size());

                    // Lut all the points
                    for(auto& p : pts) {
                        l.push_back(lut(image(p[0], p[1])));
                    }

                    auto segments = utility::vision::quexClassify<quex::FineObjectLexer>(l.begin(), l.end());
                    insertSegments(result,segments,i,true);
                    
                    
                    //XXX: horizontal scans...
                    scanOrder = arma::sort_index(f.col(1));
                    }
                }
            
            //7. emit

            return std::move(result);
        }

    }  // vision
}  // modules
