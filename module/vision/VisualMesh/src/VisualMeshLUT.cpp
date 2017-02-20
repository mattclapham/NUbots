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
    , minAngleJump(minAngleJump)   
    , requests()
    , luts(slices) {}

    double deltaPhi(const message::vision::MeshObjectRequest& request, double phi, double height) {

        switch(request.type.value) {
            case MeshObjectRequest::Type::CIRCLE:
                return Circle::deltaPhi(request, phi, height);

            case MeshObjectRequest::Type::SPHERE:
                return Sphere::deltaPhi(request, phi, height);

            case MeshObjectRequest::Type::CYLINDER:
                return Cylinder::deltaPhi(request, phi, height);

            default:
                return std::numeric_limits<double>::max();
        }
    }

    double deltaTheta(const message::vision::MeshObjectRequest& request, double phi, double height) {
        switch(request.type.value) {
            case MeshObjectRequest::Type::CIRCLE:
                return Circle::deltaTheta(request, phi, height);

            case MeshObjectRequest::Type::SPHERE:
                return Sphere::deltaTheta(request, phi, height);

            case MeshObjectRequest::Type::CYLINDER:
                return Cylinder::deltaTheta(request, phi, height);

            default:
                return std::numeric_limits<double>::max();
        }
    }

    void VisualMeshLUT::regenerate() {

        // We need a minimum jump set, otherwise ignore
        if (minAngleJump < 0) {
            std::cout << "NOT RUNNING REGENERATE!!!" << std::endl;
            return;
        }

        // Our new set of LUTs for each height
        std::vector<LUTSet> newLUTs(slices);

        // Loop through each of our height slices
        for(size_t i = 0; i < slices; ++i) {

            // Get our components of our LUT
            auto& lut = newLUTs[i];
            auto& rowDeltas = lut.rowDeltas;
            auto& rows = lut.rows;
            //auto& edges = lut.edges;

            // Calculate our camera height
            double height = minHeight + double(i) * ((maxHeight - minHeight) / (double(slices) - 1));

            // Scoping for calculating the phi/dtheta lut
            {
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
                        rowDeltas.push_back(std::make_pair(phi, minThetaJump));
                        //std::cout << "just pushed rowDelta" << std::endl;
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
                        rowDeltas.push_back(std::make_pair(phi, minThetaJump));
                        //std::cout << "just pushed ANOTHER rowDelta" << std::endl;
                    }
                    phi -= minPhiJump;
                }

                // Sort the list so it goes from lowest to highest phi
                std::sort(std::begin(rowDeltas), std::end(rowDeltas));
            }

            /*
                CALCULATE THE ROW LUT FOR THE EDGE GRAPH   
             */

            /* Scope for row calculation */ 
            if(false){

                // We will need *2-1 the number of elements for the rows (one additional for each pair)
                std::cout << "rows size: " << rows.size() << "rowDeltas size: " << rowDeltas.size() << std::endl;

                rows.reserve(rowDeltas.size() * 2 - 1);

                // Build the rest of our rows starting after the first row
                int currentIndex = int(M_PI / rowDeltas.front().second) * 2;
                int prevSize = currentIndex;

                // First row starts at 0
                // Add our first horizontal row
                rows.emplace_back(Row {
                    0,
                    currentIndex,
                    { rowDeltas.front().first,  rowDeltas.front().first },
                    { rowDeltas.front().second, rowDeltas.front().second }
                });

                for (auto it = rowDeltas.begin() + 1; it != rowDeltas.end(); ++it) {

                    // Current row's horizontal size
                    int nextSize = int(M_PI / it->second) * 2;

                    // Add our diagonal row
                    rows.emplace_back(Row {
                        currentIndex,
                        currentIndex + prevSize + nextSize,
                        { rows.back().phis[0], it->first },
                        { rows.back().dThetas[0], it->second }
                    });

                    // Advance our index from the diagonal
                    currentIndex += prevSize + nextSize;

                    // Add our horizontal row
                    rows.emplace_back(Row {
                        currentIndex,
                        currentIndex + nextSize,
                        { it->first, it->first},
                        { it->second, it->second }
                    });

                    // Advance our index
                    currentIndex += nextSize;

                    // Set our previous size
                    prevSize = nextSize;
                }

            }

            /* Scope for the edge graph calculation */ if(false){

                // Allocate enough storage space to store all our edges
                std::vector<Edge> edges(rows.back().end);

                // 3 Coordinate systems are used in the next section for creating graphs
                // Centred:  (c prefix) is the offset from the centre of a row
                // Local:    (l prefix) is local to a single row from the left
                // Global:   (g prefix) is the index across the entire array
                // Relative: (r prefix) is the offset from the current index to the target

                // Loop through the horizontal lines
                for (uint i = 0; i < rows.size(); i += 2) {

                    // Row's half horizontal size
                    int halfSize = int(M_PI / rows[i].dThetas[0]);

                    // Loop through theta
                    for (int lI = 0; lI < halfSize * 2; ++lI) {

                        const int gI = lI + rows[i].begin;
                        const int cI = lI - halfSize;

                        // We are symmetrical around the axis so we subtract half the size
                        double theta1 =  cI      * rows[i].dThetas[0];
                        double theta2 = (cI + 1) * rows[i].dThetas[0];

                        // Values we need to calculate unit vectors
                        double sinPhi = std::sin(rows[i].phis[0]);
                        double cosPhi = std::cos(rows[i].phis[0]);
                        double sinTheta1 = std::sin(theta1);
                        double cosTheta1 = std::cos(theta1);
                        double sinTheta2 = std::sin(theta2);
                        double cosTheta2 = std::cos(theta2);

                        // Now we convert the point to a much more useful unit vector pair
                        edges[gI].p1 = arma::vec3({ cosTheta1 * sinPhi, sinTheta1 * sinPhi, -cosPhi });
                        edges[gI].p2 = arma::vec3({ cosTheta2 * sinPhi, sinTheta2 * sinPhi, -cosPhi });
                    }
                }

                // Loop through the diagonal lines
                for (uint i = 1; i < rows.size(); i += 2) {

                    // Get our global indicies for row starts
                    const int& gRowAbove = rows[i - 1].begin;
                    const int& gRow = rows[i].begin;
                    const int& gRowBelow = rows[i + 1].begin;

                    // Calculate our row sizes for our row and the rows around us
                    const int halfRowAboveSize = int(M_PI / rows[i].dThetas[0]);
                    const int halfRowBelowSize = int(M_PI / rows[i].dThetas[1]);
                    const int halfRowSize = halfRowAboveSize + halfRowBelowSize;

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
                                            ? (gRowBelow + lPrevBelowI - gI)
                                            : (gRowAbove + lPrevAboveI - gI);

                        int rNextLink = lNextAboveI == lAboveI
                                            ? (gRowBelow + lBelowI - gI)
                                            : (gRowAbove + lAboveI - gI);

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
            }
        }
        luts = std::move(newLUTs);
    }

    void VisualMeshLUT::setMinimumJump(const double& jump) {
        std::cout << "jump: " << jump << std::endl;
        minAngleJump = jump;
        regenerate();
    }

    void VisualMeshLUT::addShape(const MeshObjectRequest& request) {
        std::cout << "adding a shape" << std::endl;

        // Add the request to our list
        requests.push_back(request);

        // Regenerate our LUT
        regenerate();
    }

}
}
