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

#ifndef MODULE_VISION_VISUALMESHLUT_H
#define MODULE_VISION_VISUALMESHLUT_H

#include "message/vision/MeshObjectRequest.h"

#include <utility>
#include <vector>
#include <cstddef>

#include <armadillo>

namespace module {
namespace vision {

    class VisualMeshLUT {
    public:


        struct Row {
            Row() : begin(), end(), phis(), dThetas() {}
            Row(int begin, int end, arma::vec2 phis, arma::vec3 dThetas) : begin(begin), end(end), phis(phis), dThetas(dThetas) {}
            int begin;
            int end;
            arma::vec2 phis;
            arma::vec2 dThetas;
        };

        struct Edge {
            Edge() : p1(), p2(), connections() {}
            Edge(arma::vec3 p1, arma::vec3 p2, std::array<int,4> connections) : p1(p1), p2(p2), connections(connections) {}
            /// The upper right point in this edge
            arma::vec3 p1;
            /// The lower left point in this edge
            arma::vec3 p2;

            // The relative offset to the indicies of the four closest edges
            std::array<int, 4> connections;
        };

        struct LUTSet {
            LUTSet() : rowDeltas(), rows(), edges() {}
            LUTSet(std::vector<std::pair<double, double>> rowDeltas, std::vector<Row> rows, std::vector<Edge> edges) : rowDeltas(rowDeltas), rows(rows), edges(edges) {}

            std::vector<std::pair<double, double>> rowDeltas;
            std::vector<Row> rows;
            std::vector<Edge> edges;
        };

        VisualMeshLUT(double minHeight, double maxHeight, int slices = 50, double minAngleJump = -1);

        void addShape(const message::vision::MeshObjectRequest& request);

        void setMinimumJump(const double& minimumAngleJump);

        void regenerate();

        template <typename TFunc>
        std::vector<std::pair<double, double>> lookup(double height, double minPhi, double maxPhi, TFunc&& thetaFunction) {
            // Get lutset for this height
            uint index = uint(((height - minHeight) / (maxHeight - minHeight)) * (slices - 1)); //will hopefully never be negative
            auto& lut = luts[index < 0 ? 0 : index >= slices ? slices - 1 : index];
            int total = 0;
            for(LUTSet l : luts){
                total+= l.rowDeltas.size();
            }

            //std::cout << "luts row deltas: " << total << std::endl;
            //std::cout << "lut row deltas after index slicing: " << lut.rowDeltas.size() << std::endl;         
            // Make a row to compare to
            LUTSet comparisonRowDeltas;
            comparisonRowDeltas.rowDeltas = { std::make_pair(minPhi, 0), std::make_pair(maxPhi, 0) };

            //std::cout << "Min Phi" << minPhi << std::endl;
            //std::cout << "Max Phi" << maxPhi << std::endl;


            // for (auto& r : lut.rowDeltas) {
            //     std::cout << "rowDelta first: " << r.first << std::endl;
            // }

            // Do a binary search for the min/max row in this lut
            auto start = std::lower_bound(lut.rowDeltas.begin(), lut.rowDeltas.end(), comparisonRowDeltas.rowDeltas[0], [] (const std::pair<double,double>& a, const std::pair<double,double>& b) {
                return a.first < b.first;
            });

            auto end = std::upper_bound(lut.rowDeltas.begin(), lut.rowDeltas.end(), comparisonRowDeltas.rowDeltas[1], [] (const std::pair<double,double>& a, const std::pair<double,double>& b) {
                return a.first < b.first;
            });

            // Calculate the min/max theta for each row in range and the corresponding index
            std::vector<std::pair<double, double>> output;
            //std::cout << "start: " << start << " end: " << end << std::endl;
            for (auto it = start; it != end; ++it) {
                auto& row = *it;
                // Calculate our min and max theta values for each phi
                std::vector<std::pair<float, float>> thetas = thetaFunction(row.first);
                float minTheta = thetas[0].first;
                float maxTheta = thetas[0].second;
                if(minTheta > maxTheta){ //can do this better
                    float temp = maxTheta;
                    maxTheta = minTheta;
                    minTheta = temp;
                }

                float phi = row.first;
                float rowDelta = row.second;
                // Work out what index these theta values correspond to
                float currentTheta = 0;
                while(currentTheta < maxTheta){
                    output.push_back(std::make_pair(phi, currentTheta));
                    currentTheta += rowDelta;
                }
                currentTheta = 0 - rowDelta;
                while(currentTheta > minTheta){
                    output.push_back(std::make_pair(phi, currentTheta));
                    currentTheta -= rowDelta;
                } 
            }
            // Return an array of points for each sample point in the LUT
            NUClear::log("lookup output size: ", output.size());
            return output;
        }

    private:
        size_t slices;
        double minHeight;
        double maxHeight;
        double minAngleJump;
        std::vector<message::vision::MeshObjectRequest> requests;

        std::vector<LUTSet> luts;
    };

}
}

#endif  // MODULE_VISION_VISUALMESHLUT_H
