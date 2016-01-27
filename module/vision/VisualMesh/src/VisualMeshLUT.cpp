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

    VisualMeshLUT::VisualMeshLUT(double minHeight, double maxHeight, int slices)
    : slices(slices)
    , minHeight(minHeight)
    , maxHeight(maxHeight) {}

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
                    minPhiJump = std::min(minPhiJump, phiJump);
                    minThetaJump = std::min(minThetaJump, thetaJump);
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
                    minPhiJump = std::min(minPhiJump, phiJump);
                    minThetaJump = std::min(minThetaJump, thetaJump);
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

        luts = newLUTs;
    }

    void VisualMeshLUT::addShape(const MeshObjectRequest& request) {

        // Add the request to our list
        requests.push_back(request);

        // Regenerate our LUT
        regenerate();
    }

    std::pair<std::vector<std::pair<double, double>>::iterator, std::vector<std::pair<double, double>>::iterator> VisualMeshLUT::getLUT(double height, double minPhi, double maxPhi) {

        auto& lut = luts[int(((height - minHeight) / (maxHeight - minHeight)) * (slices - 1))];
        std::cout << "height" << height << std::endl;
        std::cout <<"index = " << (((height - minHeight) / (maxHeight - minHeight)) * (slices - 1)) << std::endl;
        // Get the phi values we are interested in
        auto start = std::lower_bound(lut.begin(), lut.end(), std::make_pair(minPhi, minPhi));
        auto end = std::upper_bound(start, lut.end(), std::make_pair(maxPhi, maxPhi));

        // Return our pair of iterators
        return std::make_pair(start, end);
    }

}
}