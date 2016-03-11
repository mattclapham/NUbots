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

        // Loop through each of our heights in the lut
        for (auto& h : newLUTs) {

            std::vector<Edge> edges;
            edges.reserve(100000);

            // Loop through each phi/theta values and add our cross lines
            for(auto& row : h) {
                for (float theta = -(M_PI_2 - std::fmod(M_PI_2, row.second)); theta < (M_PI_2 - row.second); theta += row.second) {
                    Edge edge;
                    edge.p1 = { row.first, theta };
                    edge.p2 = { row.first, theta + row.second };

                    edges.push_back(edge);
                }
            }

            // Loop our phi/theta values as pairs
            for(int i = 1; i < h.size(); ++i) {

                // Get our two phi lines
                // Line 1 will always have the greater phi value
                auto& line1 = h[i - 1];
                auto& line2 = h[i];

                // These hold the points we generate, phi is constant
                arma::vec2 p1 = { line1.first, -(M_PI_2 - std::fmod(M_PI_2, line1.second)) };
                arma::vec2 p2 = { line2.first, -(M_PI_2 - std::fmod(M_PI_2, line1.second)) };

                // We want to loop using the smaller theta
                // To do this we change where our references point so they
                // apply and use values in the correct spots.
                // We can then copy p1 and p2 and they will be in the correct order
                auto& thetaSmall = line1.second <= line2.second ? p1[1] : p2[1];
                auto& thetaBig   = line1.second >  line2.second ? p1[1] : p2[1];

                double dThetaSmall = std::min(line1.second, line2.second);
                double dThetaBig   = std::max(line1.second, line2.second);

                // We loop until our next movement in the small theta would go too far
                while((thetaSmall + dThetaSmall) < M_PI_2) {

                    // Create our edge
                    Edge edge;
                    edge.p1 = p1;
                    edge.p2 = p2;
                    edges.push_back(std::move(edge));

                    // If we have moved over to the next major theta
                    if (thetaSmall > (thetaBig + dThetaBig * 0.5)) {
                        thetaBig += dThetaBig;
                    }
                    // Otherwise we moved to the next minor theta
                    else {
                        thetaSmall += dThetaSmall;
                    }
                }
            }

            // The graph is sorted now based on phi1 and then by the theta values
            // It is sorted by both thetas always as both sets are always monotonic

            // // Now we make a new list to calculate the edges
            // std::vector<std::vector<Edge>::iterator> edgeSort;
            // for(auto it = edges.begin(); it != edges.end(); ++it) {
            //     edgeSort.push_back(it);
            // }

            // TODO FIND A WAY TO LINK UP THE GRAPH
        }

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
