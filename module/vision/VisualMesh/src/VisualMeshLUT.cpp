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

            std::cout << values.size() << std::endl;

            // This is our height for this index
            newLUTs.push_back(values);
        }

        std::cout << "Done Points!" << std::endl;

        // Loop through each of our heights in the lut
        for (auto& h : newLUTs) {

            struct Row {
                int start;
                arma::vec2 phis;
                arma::vec2 dThetas;
            };

            // First row starts at 0
            std::vector<Row> rows;
            rows.reserve(h.size() - 1);

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

            std::cout << currentIndex << std::endl;

            // Allocate enough storage space to store all our edges
            std::vector<Edge> edges(currentIndex);


            // 3 Coordinate systems are used in the next section for creating graphs
            // Local:    (l prefix) is local to a single row
            // Global:   (g prefix) is the index across the entire array
            // Relative: (r prefix) is the offset from the current index to the target

            // TODO deal with the fact that the list is mirrored

            // Loop through the horizontal lines
            for (int i = 0; i < rows.size(); i += 2) {

                // Row's half horizontal size
                int size = int(M_PI / rows[i].dThetas[0]);

                // Loop through theta
                for (int lI = 0; lI < size * 2; ++lI) {

                    int gI = lI + rows[i].start;

                    // We are symmetrical around the axis so we subtract half the size
                    double theta1 = (lI - size)     * rows[i].dThetas[0];
                    double theta2 = (lI - size + 1) * rows[i].dThetas[0];

                    edges[gI].p1 = { rows[i].phis[0], theta1 };
                    edges[gI].p2 = { rows[i].phis[0], theta2 };
                }
            }

            // Loop through the diagonal lines
            for (int i = 1; i < rows.size(); i += 2) {

                // Get our global indicies for row starts
                const int& gPrevRow = rows[i - 1].start;
                const int& gCurrRow = rows[i].start;
                const int& gNextRow = rows[i + 1].start;

                // Work out the size of our region (so we can find the centre)
                const int centreOffset = (gNextRow - gCurrRow) / 2;
                std::cout << (gNextRow - gCurrRow) << std::endl;

                // Calculate our relative row starts
                const int rPrevRow = gCurrRow - gPrevRow;
                const int rNextRow = gCurrRow - gNextRow;

                // The horizontal movement ratio
                const double hRatio = rows[i].dThetas[1] / (rows[i].dThetas[0] + rows[i].dThetas[1]);

                // Loop through our diagonal indicies
                for (int gI = gCurrRow; gI < gNextRow; ++gI) {

                    // Get our local index
                    const int lI = gI - gCurrRow;

                    // Calculate our  indicies for above and below
                    const int lAboveI = lI * hRatio;
                    const int lBelowI = lI - lAboveI;
                    const int gAboveI = gPrevRow + lAboveI;
                    const int gBelowI = gNextRow + lBelowI;

                    // Calculate our indexes before and after to see how we move
                    // TODO if the edges look offset round here
                    const int lPrevAboveI = int((lI - 1) * hRatio);
                    const int lPrevBelowI = ((lI - 1) - lPrevAboveI);
                    const int lNextAboveI = int((lI + 1) * hRatio);

                    // Check which rows above/below we should link too
                    // We check if left/right edge is above or below us
                    int rPrevLink = lPrevAboveI == lAboveI
                                        ? rNextRow + lPrevBelowI
                                        : rPrevRow + lPrevAboveI;

                    int rNextLink = lNextAboveI != lAboveI
                                        ? rNextLink = rNextRow + lBelowI
                                        : rNextLink = rPrevRow + lAboveI;

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

                    // We need to reverse the links as well
                    // It will connect NW NE SE SW
                    edges[gI + rPrevLink].connections[rPrevLink > 0 ? 0 : 2] = -rPrevLink;
                    edges[gI + rNextLink].connections[rNextLink > 0 ? 1 : 3] = -rNextLink;

                }
            }

            // // Generate the diagonal lines
            // for (int lI = 0; lI <= (int(M_PI / dTheta1) + int(M_PI / dTheta2)) * 2; ++lI) {

            //     // Get our global indicies for row starts
            //     int gCurrRow = somemagichere;
            //     int gPrevRow = somemagichere;
            //     int gNextRow = somemagichere;

            //     // Calculate our relative row starts
            //     int rPrevRow = gRow - gPrevRow;
            //     int rNextRow = gRow - gNextRow;

            //     // Calculate our global index
            //     int gI = i + gRow;

            //     // The horizontal movement ratio
            //     double hRatio = theta2 / (theta1 + theta2);

            //     // Calculate our index above and below
            //     int lAboveI = lI * hRatio;
            //     int lBelowI = lI - lAboveI;

            //     // Calculate our indexes before and after to see how we move
            //     int lPrevAboveI = int((lI - 1) * hRatio); // TODO if the edges look offset round here
            //     int lNextAboveI = int((lI + 1) * hRatio);

            //     if (lPrevAboveI != lAboveI) {
            //         // Above has moved on the left
            //         rPrevLink = rPrevRow + lPrevAboveI;
            //     }
            //     else {
            //         // Below has moved on the left
            //         rPrevLink = rNextRow + ((index - 1) - lPrevAboveI);
            //     }

            //     if (lNextAboveI != lAboveI) {
            //         // Above has moved on the right
            //         rNextLink = rPrevRow + lAboveI;
            //     }
            //     else {
            //         // Below has moved to the right
            //         rNextLink = rNextRow + lBelowI;
            //     }

            //     // Set our points from our above/below points
            //     edges[gI].p1 = edges[above].p1;
            //     edges[gI].p2 = edges[below].p1;

            //     // Calculate ours next previous index
            //     edges[gI].connections[0] = +1;
            //     edges[gI].connections[1] = -1;
            //     edges[gI].connections[2] = rPrevLink; // Which one changed from previous
            //     edges[gI].connections[3] = rNextLink; // Which one changed to next

            //     // Link those ones
            //     edges[gI + rPrevLink].connections[rPrevLink > 0 ? 0 : 2] = todoindexrelativetothem; // TODO this is wrong!! links should be based on prev/next and something something
            //     edges[gI + rNextLink].connections[rNextLink > 0 ? 1 : 3] = todoindexrelativetothem;
            // }





            // its int(M_PI/theta) * 2 for each horizontal line
            // and then added for each pair
            // so times 2 - first and last

            // // We have one row for each single phi, and one for each phi pair
            // std::vector<Edge> edges(h.size() * 2 - 1);

            // // Generate the values for the same phi edges
            // for (size_t i = 0; i < h.size(); ++i) {

            //     // Loop through our theta values to make our edges
            //     for (double theta = -(M_PI_2 - std::fmod(M_PI_2, h[i].second));
            //          theta < (M_PI_2 - h[i].second);
            //          theta += h[i].second) {

            //         Edge edge;
            //         edge.p1 = { h[i].first, theta };
            //         edge.p2 = { h[i].first, theta + h[i].second };

            //         edges[i * 2].push_back(edge);
            //     }
            // }

            // std::cout << "done generating the horizontal lines!" << std::endl;

            // Now we generate the pairs and the edges


            // indexSmall = indexofdiagonal - indexLarge


            // for (auto& row : h) {

            // }



            // // For each phi/dTheta there are round(M_PI/dTheta) horizontal lines

            // // For each phi/dTheta pair there are round(M_PI/dTheta1) + round(M_PI/dTheta2) points

            // // Given the index of an edge in the theta pairs (between 0 and round(M_PI/dTheta1) + round(M_PI/dTheta2) find the index of the edges it connects to




            // std::vector<Edge> edges;
            // edges.reserve(100000);

            // // Loop through each phi/theta values and add our cross lines
            // for(auto& row : h) {
            //     for (float theta = -(M_PI_2 - std::fmod(M_PI_2, row.second)); theta < (M_PI_2 - row.second); theta += row.second) {
            //         Edge edge;
            //         edge.p1 = { row.first, theta };
            //         edge.p2 = { row.first, theta + row.second };

            //         edges.push_back(edge);
            //     }
            // }

            // // Loop our phi/theta values as pairs
            // for(int i = 1; i < h.size(); ++i) {

            //     // Get our two phi lines
            //     // Line 1 will always have the greater phi value
            //     auto& line1 = h[i - 1];
            //     auto& line2 = h[i];

            //     // These hold the points we generate, phi is constant
            //     arma::vec2 p1 = { line1.first, -(M_PI_2 - std::fmod(M_PI_2, line1.second)) };
            //     arma::vec2 p2 = { line2.first, -(M_PI_2 - std::fmod(M_PI_2, line1.second)) };

            //     // We want to loop using the smaller theta
            //     // To do this we change where our references point so they
            //     // apply and use values in the correct spots.
            //     // We can then copy p1 and p2 and they will be in the correct order
            //     auto& thetaSmall = line1.second <= line2.second ? p1[1] : p2[1];
            //     auto& thetaBig   = line1.second >  line2.second ? p1[1] : p2[1];

            //     double dThetaSmall = std::min(line1.second, line2.second);
            //     double dThetaBig   = std::max(line1.second, line2.second);

            //     // We loop until our next movement in the small theta would go too far
            //     while((thetaSmall + dThetaSmall) < M_PI_2) {

            //         // Create our edge
            //         Edge edge;
            //         edge.p1 = p1;
            //         edge.p2 = p2;
            //         edges.push_back(std::move(edge));

            //         // If we have moved over to the next major theta
            //         if (thetaSmall > (thetaBig + dThetaBig * 0.5)) {
            //             thetaBig += dThetaBig;
            //         }
            //         // Otherwise we moved to the next minor theta
            //         else {
            //             thetaSmall += dThetaSmall;
            //         }

            //         // TODO Edge connections
            //         // Each edge will always connect two two interphi edges, and two intraphi edges
            //     }
            // }
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
