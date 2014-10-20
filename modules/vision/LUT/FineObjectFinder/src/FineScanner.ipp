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

        template <int camID>
        arma::imat FineScanner::fineScanClassify(const std::vector<arma::ivec2>& allpts,
                                                const arma::uvec& selectedpts,
                                                const messages::input::Image<camID>& image) const {

            //collect data on our points
            arma::running_stat_vec<arma::ivec2> stats;
            for (uint i = 0; i < selectedpts.n_rows; ++i) {
                stats(allpts[selectedpts[i]]);
            }

            double radialDiameterPx = utility::vision::geometry::sphere::arcSizeFromBaseRay(
                                    bulkPixel2Ray(
                                        arma::ivec({(stats.min()[0]),
                                        (stats.min()[1])}).t(),image),
                                    MAX_OBJECT_SIZE,CAMERA_HEIGHT)[0];

            if (image.lens.type == Image<camID>::Lens::Type::RADIAL) {
                radialDiameterPx *= image.lens.parameters.radial.pitch;
            } else if (image.lens.type == Image<camID>::Lens::Type::EQUIRECTANGULAR) {
                //XXX: zen hack
                const double pitch = sqrt(image.lens.parameters.equirectangular.fov[0]*image.lens.parameters.equirectangular.fov[0] +
                             image.lens.parameters.equirectangular.fov[1]*image.lens.parameters.equirectangular.fov[1]) /
                             sqrt(image.dimensions[0]*image.dimensions[0] + image.dimensions[1]*image.dimensions[1]);
                radialDiameterPx *= pitch;
            }

            arma::ivec2 center = (stats.max() + stats.min());
            //find the diameter of the object
            int diameter = int(std::min<double>(arma::norm(arma::conv_to<arma::vec>::from(arma::ivec(stats.max() - stats.min()))),
                                radialDiameterPx) +
                                MIN_SURROUNDING_PIXELS + 0.5);

            //make horizontal lines
            int minPixel = std::max<int>(center[1]-diameter,0);
            int maxPixel = std::min<int>(center[1]+diameter,image.dimensions[1]);
            int increment = std::max(diameter/CROSSHATCH_LINES,1);

            for (int i = minPixel; i < maxPixel; i += increment) {
                int diff = int(sqrt(diameter - i*i));
                int leftX = std::max<int>(center[0]-diff, 0);
                int rightX = std::min<int>(center[0]+diff, image.dimensions[0]);
            }

            //make vertical lines
            minPixel = std::max<int>(center[0]-diameter,0);
            maxPixel = std::min<int>(center[0]+diameter,image.dimensions[0]);

            for (int i = minPixel; i < maxPixel; i += increment) {
                int diff = int(sqrt(diameter - i*i));
                int topY = std::max<int>(center[1]-diff, 0);
                int bottomY = std::min<int>(center[1]+diff, image.dimensions[1]);
            }

            //XXX: return lines
            return arma::imat();
        }

        template <int camID>
        std::map<uint,std::vector<arma::ivec2>> FineScanner::findObjects(const messages::input::Image<camID>& image,
                                                            const messages::vision::LookUpTable& lut,
                                                            const arma::mat& horizonNormals,
                                                            const std::map<uint,std::vector<arma::ivec2>>& coarseScan) const {
            //XXX: implement dynamic candidates
            std::vector<uint> candidateColours;


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
            //XXX: rewrite distances to work with mats in bulk?
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
                        arma::uvec activePixels = arma::find(pointDists[c].col(i) <= std::max(pointSizes[c][i], MIN_ANGULAR_SIZE));
                        active.rows(activePixels).fill(0);

                        fineScanLines.push_back( fineScanClassify( coarseScan.at(c), activePixels , image ) );
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
