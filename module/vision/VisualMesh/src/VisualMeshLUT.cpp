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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#include "VisualMeshLUT.h"

#include <cstddef>
#include <algorithm>
#include <iostream>

#include "Circle.h"
#include "Cylinder.h"
#include "Sphere.h"

namespace module {
namespace vision {

    using message::vision::MeshObjectRequest;

    VisualMeshLUT::VisualMeshLUT(double minHeight, double maxHeight, int slices, double minAngleJump)
    : slices(slices)
    , minHeight(minHeight)
    , maxHeight(maxHeight)
    , luts(slices)
    , minAngleJump(minAngleJump) {}

    double deltaPhi(const message::vision::MeshObjectRequest& request, double phi, double height) {

        switch(request.type) {
            case MeshObjectRequest::CIRCLE:
                return Circle::deltaPhi(request, phi, height);

            case MeshObjectRequest::SPHERE:
                return Sphere::deltaPhi(request, phi, height);

            case MeshObjectRequest::CYLINDER:
                return Cylinder::deltaPhi(request, phi, height);

            default:
                return std::numeric_limits<double>::max();
        }
    }

    double deltaTheta(const message::vision::MeshObjectRequest& request, double phi, double height) {
        switch(request.type) {
            case MeshObjectRequest::CIRCLE:
                return Circle::deltaTheta(request, phi, height);

            case MeshObjectRequest::SPHERE:
                return Sphere::deltaTheta(request, phi, height);

            case MeshObjectRequest::CYLINDER:
                return Cylinder::deltaTheta(request, phi, height);

            default:
                return std::numeric_limits<double>::max();
        }
    }

    void VisualMeshLUT::regenerate() {

        std::cout << "Regenerating!" << std::endl;

        // We need a minimum jump set, otherwise ignore
        if (minAngleJump < 0) {
            return;
        }

        // Our new luts
        std::vector<std::vector<std::pair<double, double>>> newLUTs;

        // Loop through each of our height slices
        for(size_t i = 0; i < slices; ++i) {

            double height = minHeight + double(i) * ((maxHeight - minHeight) / (double(slices) - 1));

            std::vector<std::pair<double, double>> values;

            // We loop from the ground up to the horizon first
            double phi = 0;
            while(phi < M_PI_2) {

                // Start at max
                double minPhiJump = std::numeric_limits<double>::max();
                double minThetaJump = std::numeric_limits<double>::max();

                // Loop through our shapes and get their input
                for (auto& request : requests) {

                    double phiJump = deltaPhi(request, phi, height);
                    double thetaJump = deltaTheta(request, phi, height);

                    // Find if this is the new minimum
                    minPhiJump   = std::max(minAngleJump, std::min(minPhiJump, phiJump));
                    minThetaJump = std::max(minAngleJump, std::min(minThetaJump, thetaJump));
                }

                // Add the value if we didn't jump past the horizon
                if(phi + minPhiJump < M_PI_2) {
                    values.push_back(std::make_pair(phi, minThetaJump));
                }
                phi += minPhiJump;
            }

            // Now we loop from the sky down to the horizon
            phi = M_PI;
            while(phi > M_PI_2) {

                // Start at max
                double minPhiJump = std::numeric_limits<double>::max();
                double minThetaJump = std::numeric_limits<double>::max();

                // Loop through our shapes and get their input
                for (auto& request : requests) {

                    // Get the jumps
                    double phiJump = deltaPhi(request, phi, height);
                    double thetaJump = deltaTheta(request, phi, height);

                    // Find if this is the new minimum
                    minPhiJump   = std::max(minAngleJump, std::min(minPhiJump, phiJump));
                    minThetaJump = std::max(minAngleJump, std::min(minThetaJump, thetaJump));
                }

                // Add the value if we didn't jump past the horizon
                if(phi - minPhiJump > M_PI_2) {
                    values.push_back(std::make_pair(phi, minThetaJump));
                }
                phi -= minPhiJump;
            }

            // Sort the list so it goes from lowest to highest phi
            std::sort(std::begin(values), std::end(values));

            // This is our height for this index
            newLUTs.push_back(values);
        }

        std::cout << "Done Points!" << std::endl;

        /*
            EDGE GRAPH
         */

        std::vector<std::pair<std::vector<Row>, std::vector<Edge>>> newEdgeLUTs;
        newEdgeLUTs.reserve(newLUTs.size());

        // Loop through each of our heights in the lut
        for (auto& h : newLUTs) {

            // First row starts at 0
            std::vector<Row> rows;
            rows.reserve(h.size() * 2 - 1);

            // Add our first horizontal row
            rows.emplace_back(Row {
                0,
                { h.front().first, h.front().first },
                { h.front().second, h.front().second }
            });

            // Loop through our edges and build the rest of our rows
            // starting after the first row which we already added
            int currentIndex = int(M_PI / h.front().second) * 2;
            for (auto it = h.begin() + 1; it != h.end(); ++it) {

                // Previous horizontal row size
                int prevSize = int(M_PI / rows.back().dThetas[0]) * 2;

                // Current row's horizontal size
                int nextSize = int(M_PI / it->second) * 2;

                // Add our diagonal row
                rows.emplace_back(Row {
                    currentIndex,
                    { rows.back().phis[0], it->first },
                    { rows.back().dThetas[0], it->second }
                });

                // Advance our index from the diagonal
                currentIndex += prevSize + nextSize;

                // Add our horizontal row
                rows.emplace_back(Row {
                    currentIndex,
                    { it->first, it->first},
                    { it->second, it->second }
                });

                // Advance our index
                currentIndex += nextSize;
            }

            // Allocate enough storage space to store all our edges
            std::vector<Edge> edges(currentIndex);

            // 3 Coordinate systems are used in the next section for creating graphs
            // Centred:  (c prefix) is the offset from the centre of a row
            // Local:    (l prefix) is local to a single row from the left
            // Global:   (g prefix) is the index across the entire array
            // Relative: (r prefix) is the offset from the current index to the target

            // Loop through the horizontal lines
            for (int i = 0; i < rows.size(); i += 2) {

                // Row's half horizontal size
                int halfSize = int(M_PI / rows[i].dThetas[0]);

                // Loop through theta
                for (int lI = 0; lI < halfSize * 2; ++lI) {

                    const int gI = lI + rows[i].start;
                    const int cI = lI - halfSize;

                    // We are symmetrical around the axis so we subtract half the size
                    double theta1 =  cI      * rows[i].dThetas[0];
                    double theta2 = (cI + 1) * rows[i].dThetas[0];

                    edges[gI].p1 = { rows[i].phis[0], theta1 };
                    edges[gI].p2 = { rows[i].phis[0], theta2 };
                }
            }

            // Loop through the diagonal lines
            for (int i = 1; i < rows.size(); i += 2) {

                // Get our global indicies for row starts
                const int& gRowAbove = rows[i - 1].start;
                const int& gRow = rows[i].start;
                const int& gRowBelow = rows[i + 1].start;

                // Calculate our row sizes for our row and the rows around us
                const int halfRowAboveSize = int(M_PI / rows[i].dThetas[0]);
                const int halfRowBelowSize = int(M_PI / rows[i].dThetas[1]);
                const int halfRowSize = halfRowAboveSize + halfRowBelowSize;

                // Calculate our relative row starts
                const int rRowAbove = gRowAbove - gRow;
                const int rRowBelow = gRowBelow - gRow;

                // The horizontal movement ratio (how fast the relative indicies move)
                const double hRatio = rows[i].dThetas[1] / (rows[i].dThetas[0] + rows[i].dThetas[1]);

                // Loop through our indicies for this diagonal row
                for (int gI = gRow; gI < gRowBelow; ++gI) {

                    // Get our local index and centred index
                    const int lI = gI - gRow;
                    const int cI = lI - halfRowSize;

                    // Calculate our indicies for above and below
                    const int cAboveI = cI * hRatio;
                    const int lAboveI = cAboveI + halfRowAboveSize;
                    const int lBelowI = lI - lAboveI;
                    const int gAboveI = gRowAbove + lAboveI;
                    const int gBelowI = gRowBelow + lBelowI;

                    // Calculate our indexes before and after to see how we move
                    const int cPrevAboveI = int((cI - 1) * hRatio);
                    const int cNextAboveI = int((cI + 1) * hRatio);
                    const int lPrevAboveI = cPrevAboveI + halfRowAboveSize;
                    const int lPrevBelowI = (lI - 1) - lAboveI;
                    const int lNextAboveI = cNextAboveI + halfRowAboveSize;

                    // Check which rows above/below we should link too
                    // We check if left/right edge is above or below us
                    int rPrevLink = lPrevAboveI == lAboveI
                                        ? rRowBelow + lPrevBelowI
                                        : rRowAbove + lPrevAboveI;

                    int rNextLink = lNextAboveI == lAboveI
                                        ? rNextLink = rRowBelow + lBelowI
                                        : rNextLink = rRowAbove + lAboveI;

                    // Set our points from our above/below points
                    edges[gI].p1 = edges[gAboveI].p1;
                    edges[gI].p2 = edges[gBelowI].p1;

                    // Calculate ours next previous index
                    // The first two will always be the adjacent points on this line
                    edges[gI].connections[0] = +1;
                    edges[gI].connections[1] = -1;

                    // Next are the two left/right horizontal lines we connect to
                    edges[gI].connections[2] = rPrevLink; // Which one changed from previous
                    edges[gI].connections[3] = rNextLink; // Which one changed to next

                    // // We need to reverse the links as well
                    edges[gI + rPrevLink].connections[rPrevLink > 0 ? 0 : 2] = -rPrevLink;
                    edges[gI + rNextLink].connections[rNextLink > 0 ? 1 : 3] = -rNextLink;

                }
            }

            newEdgeLUTs.emplace_back(std::make_pair(std::move(rows), std::move(edges)));
        }

        std::cout << "Done Graph!" << std::endl;

        luts = newLUTs;
    }

    void VisualMeshLUT::setMinimumJump(const double& jump) {
        minAngleJump = jump;
        regenerate();
    }

    void VisualMeshLUT::addShape(const MeshObjectRequest& request) {

        // Add the request to our list
        requests.push_back(request);

        // Regenerate our LUT
        regenerate();
    }

    std::pair<std::vector<std::pair<double, double>>::iterator, std::vector<std::pair<double, double>>::iterator> VisualMeshLUT::getLUT(double height, double minPhi, double maxPhi) {

        int index = int(((height - minHeight) / (maxHeight - minHeight)) * (slices - 1));
        auto& lut = luts[index < 0 ? 0 : index >= slices ? slices - 1 : index];

        // Get the phi values we are interested in
        auto start = std::lower_bound(lut.begin(), lut.end(), std::make_pair(minPhi, minPhi));
        auto end = std::upper_bound(start, lut.end(), std::make_pair(maxPhi, maxPhi));

        // Return our pair of iterators
        return std::make_pair(start, end);
    }

}
}
