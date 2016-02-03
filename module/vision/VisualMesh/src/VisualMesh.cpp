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

#include "VisualMesh.h"
#include "message/input/Image.h"
#include "message/input/CameraParameters.h"
#include "message/input/Sensors.h"
#include "message/vision/MeshObjectRequest.h"
#include "message/support/Configuration.h"
#include "message/motion/ServoTarget.h"

#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/vision.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"

#include "utility/motion/RobotModels.h"

namespace module {
namespace vision {

    using message::input::Image;
    using message::input::CameraParameters;
    using message::input::Sensors;
    using message::vision::MeshObjectRequest;
    using message::support::Configuration;

    using utility::motion::kinematics::DarwinModel;
    using message::motion::ServoTarget;

    using utility::math::matrix::Rotation3D;
    using utility::nubugger::graph;
    using utility::nubugger::drawVisionLines;

    // http://www.math-only-math.com/a-cos-theta-plus-b-sin-theta-equals-c.html
    arma::vec solveAcosThetaPlusBsinThetaEqualsC(double a, double b, double c) {
        double r = std::sqrt(a*a + b*b);
        if(std::abs(c) <= r) {
            double alpha = std::atan2(b, a);
            double beta = std::acos(c/r);
            if(beta == 0) {
                return arma::vec({ alpha + beta });
            } else {
                return arma::vec({ alpha + beta, alpha - beta });
            }
        } else {
            return arma::vec();
        }
    }

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , lut(0.1, 0.5) { // TODO make this based of the kinematics

        on<Configuration>("VisualMesh.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file VisualMesh.yaml
        });

        auto sphere = std::make_unique<MeshObjectRequest>();
        sphere->type = MeshObjectRequest::SPHERE;
        sphere->radius = 0.07;
        sphere->height = 0;
        sphere->intersections = 3;
        sphere->maxDistance = 10;
        sphere->hardLimit = false;

        auto cylinder = std::make_unique<MeshObjectRequest>();
        cylinder->type = MeshObjectRequest::CYLINDER;
        cylinder->radius = 0.05;
        cylinder->height = 2;
        cylinder->intersections = 2;
        cylinder->maxDistance = 10;
        cylinder->hardLimit = false;

        auto circle = std::make_unique<MeshObjectRequest>();
        circle->type = MeshObjectRequest::CIRCLE;
        circle->radius = 0.05;
        circle->height = 0;
        circle->intersections = 2;
        circle->maxDistance = 2;
        circle->hardLimit = false;

        emit<Scope::INITIALIZE>(sphere);
        emit<Scope::INITIALIZE>(cylinder);
        emit<Scope::INITIALIZE>(circle);

        on<Trigger<MeshObjectRequest>>().then([this] (const MeshObjectRequest& request) {
            lut.addShape(request);
        });

        on<Trigger<Sensors>, With<CameraParameters>, Single>().then([this] (const Sensors& sensors, const CameraParameters& params) {

            // get field of view

            float fovX = params.FOV[0];
            float fovY = params.FOV[1];
            float focalLengthPixels = params.focalLengthPixels;

            // Camera height is z component of the transformation matrix
            double cameraHeight = sensors.orientationCamToGround(2, 3);

            // Remove yaw from our cameras rotation matrix
            Rotation3D camToGround = Rotation3D::createRotationZ(-sensors.orientationCamToGround.rotation().yaw()) * sensors.orientationCamToGround.rotation();

            /***************************
             * Calculate image corners *
             ***************************/
            // Get the corners of the view port in cam space
            double yMax = std::tan(fovX * 0.5);
            double zMax = std::tan(fovY * 0.5);
            arma::mat::fixed<3,4> cornerPointsCam = {
                1,  yMax,  zMax,
                1, -yMax,  zMax,
                1, -yMax, -zMax,
                1,  yMax, -zMax
            };

            // Normalise our cam space vectors
            cornerPointsCam /= arma::norm(cornerPointsCam.col(0));

            // Rotate the camera points into world space
            arma::mat::fixed<3,4> cornerPointsWorld = camToGround * cornerPointsCam;

            /**************************************************
             * Calculate screen edge planes and corner angles *
             **************************************************/
            arma::cube::fixed<3,3,4> screenEdgeMatricies;
            arma::vec4 screenEdgeArcs;
            for(int i = 0; i < screenEdgeMatricies.n_slices; ++i) {
                // Make our screen edge rotation matrix
                screenEdgeMatricies.slice(i).col(0) = cornerPointsWorld.col(i);
                screenEdgeMatricies.slice(i).col(2) = arma::normalise(arma::cross(cornerPointsWorld.col(i), cornerPointsWorld.col((i + 1) % 4)));
                screenEdgeMatricies.slice(i).col(1) = arma::cross(screenEdgeMatricies.slice(i).col(2), screenEdgeMatricies.slice(i).col(0));

                // Find the end of our screen edge arc
                arma::vec3 p = screenEdgeMatricies.slice(i).t() * cornerPointsWorld.col((i + 1) % 4);
                screenEdgeArcs[i] = std::atan2(p[1], p[0]);
            }

            /*************************
             * Calculate min/max phi *
             *************************/
            // Because our matrix has no yaw, we can directly extract cos and sin
            const double& cosRoll = camToGround(1, 1);
            const double& sinRoll = camToGround(2, 1);

            // Phi field of view comes from the vectors from the centre to the corner
            // There are two of these, so we chose the larger of the two
            double fovPhi = std::atan(std::max(std::abs(yMax * sinRoll + zMax * cosRoll), std::abs(yMax * sinRoll - zMax * cosRoll)));
            double fovOffset = M_PI_2 - std::acos(camToGround(0, 0));

            double minPhi = fovOffset - fovPhi;
            double maxPhi = fovOffset + fovPhi;

            /*************************************************************
             * Get our lookup table and loop through our phi/theta pairs *
             *************************************************************/
            auto phiIterator = lut.getLUT(cameraHeight, minPhi, maxPhi);
            std::vector<arma::vec3> camPoints;

            for(auto it = phiIterator.first; it != phiIterator.second; ++it) {

                const double& phi = it->first;
                const double& dTheta = it->second;

                /*********************************************************
                 * Calculate our min and max theta values for each plane *
                 *********************************************************/
                double sinPhi = sin(phi);
                double cosPhi = cos(phi);

                std::vector<double> thetaLimits;
                thetaLimits.reserve(4);

                for(int i = 0; i < 4; ++i) {

                    if (minPhi < phi && phi < maxPhi) {

                        // Get the unit vector that describes the normal to the screen plane
                        double& x = screenEdgeMatricies(0, 2, i);
                        double& y = screenEdgeMatricies(1, 2, i);
                        double& z = screenEdgeMatricies(2, 2, i);

                        // Solve our equation to find our theta intercepts
                        arma::vec v = solveAcosThetaPlusBsinThetaEqualsC(sinPhi * x, sinPhi * y, cosPhi * z);

                        // We only care about the case with two solutions
                        if(v.n_elem == 2) {

                            arma::vec2 cosV = arma::cos(v);
                            arma::vec2 sinV = arma::sin(v);

                            // Calculate a unit vector to this solution and rotate it
                            // into the screen edge plane space (removing the z component)
                            arma::vec3 p1 = { cosV[0] * sinPhi, sinV[0] * sinPhi, -cosPhi };
                            arma::vec3 p2 = { cosV[1] * sinPhi, sinV[1] * sinPhi, -cosPhi };
                            p1 = screenEdgeMatricies.slice(i).t() * p1;
                            p2 = screenEdgeMatricies.slice(i).t() * p2;

                            // Do an atan2 to find out how far around this solution is
                            double p1V = atan2(p1[1], p1[0]);
                            double p2V = atan2(p2[1], p2[0]);

                            // Check solution 1 is between 0 and our other point
                            if (0 < p1V && p1V < screenEdgeArcs[i]) {
                                thetaLimits.push_back(v[0]);
                            }

                            // Check solution 2 is between 0 and our other point
                            if (0 < p2V && p2V < screenEdgeArcs[i]) {
                                thetaLimits.push_back(v[1]);
                            }
                        }
                    }
                }

                // Sort the limits to make pairs
                std::sort(thetaLimits.begin(), thetaLimits.end());

                /***********************************
                 * Loop through our theta segments *
                 ***********************************/
                for (size_t i = 0; i < thetaLimits.size(); i += 2) {
                    const double& minTheta = thetaLimits[i];
                    const double& maxTheta = thetaLimits[i + 1];

                    // Loop through our valid theta range using delta theta
                    for (double theta = minTheta; theta < maxTheta; theta += dTheta) {

                        /*******************************************************************
                         * Add this phi/theta point to the list to project onto the screen *
                         *******************************************************************/
                        double cosTheta = std::cos(theta);
                        double sinTheta = std::sin(theta);

                        camPoints.push_back(camToGround.i() * arma::vec3({ cosTheta * sinPhi, sinTheta * sinPhi, -cosPhi }));
                    }
                }
            }

            /***********************************************
             * Project our phi/theta pairs onto the screen *
             ***********************************************/
            std::vector<arma::ivec2> screenPoints;

            std::vector<std::pair<arma::ivec2, arma::ivec2>> helperPoints;

            screenPoints.reserve(camPoints.size());
            for(auto& point : camPoints) {
                screenPoints.push_back(utility::math::vision::screenToImage(arma::vec2({(focalLengthPixels * point[1] / point[0]), (focalLengthPixels * point[2] / point[0])}), params.imageSizePixels));
                helperPoints.push_back(std::make_pair(screenPoints.back(), screenPoints.back() + arma::ivec2({1,1})));
            }

            emit(drawVisionLines(std::move(helperPoints)));
        });
    }
}
}