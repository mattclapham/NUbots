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

    template <bool HIGH_PRECISION = true>
    float fastSin(float x) {
        // Convert the input value to a range of -1 to 1
        x = x * (1.0f / M_PI);

        // Wrap around
        volatile float z = (x + 25165824.0f);
        x = x - (z - 25165824.0f);

        if(HIGH_PRECISION) {
            float y = x - x * fabs(x);

            const float Q = 3.1f;
            const float P = 3.6f;

            return y * (Q + P * fabs(y));
        }
        else {
            return 4.0f * (x - x * fabs(x));
        }
    }

    template <bool HIGH_PRECISION = true>
    float fastCos(float x) {
        return fastSin<HIGH_PRECISION>(x - M_PI_2);
    }

    // http://www.math-only-math.com/a-cos-theta-plus-b-sin-theta-equals-c.html
    arma::fvec solveAcosThetaPlusBsinThetaEqualsC(float a, float b, float c) {
        float r = std::sqrt(a*a + b*b);
        if(std::abs(c) <= r) {
            float alpha = std::atan2(b, a);
            float beta = std::acos(c/r);
            if(beta == 0) {
                return arma::fvec({ alpha + beta });
            } else {
                return arma::fvec({ alpha + beta, alpha - beta });
            }
        } else {
            return arma::fvec();
        }
    }

    std::vector<std::pair<float, float>> thetaLimits(float phi, const arma::fcube::fixed<3,3,4>& screenEdgeMatricies, const arma::fvec& screenEdgeArcs) {

        float sinPhi = sin(phi);
        float cosPhi = cos(phi);

        std::vector<float> thetaLimits;
        thetaLimits.reserve(4);

        // Our screen has four sides, this loops through them
        for(int i = 0; i < 4; ++i) {

            // Get the unit vector that describes the normal to the screen plane
            const float& x = screenEdgeMatricies(0, 2, i);
            const float& y = screenEdgeMatricies(1, 2, i);
            const float& z = screenEdgeMatricies(2, 2, i);

            // Solve our equation to find our theta intercepts
            arma::fvec v = solveAcosThetaPlusBsinThetaEqualsC(sinPhi * x, sinPhi * y, cosPhi * z);

            // We only care about the case with two solutions
            if(v.n_elem == 2) {

                arma::fvec2 cosV = arma::cos(v);
                arma::fvec2 sinV = arma::sin(v);

                // Calculate a unit vector to this solution and rotate it
                // into the screen edge plane space (removing the z component)
                arma::fvec3 p1 = { cosV[0] * sinPhi, sinV[0] * sinPhi, -cosPhi };
                arma::fvec3 p2 = { cosV[1] * sinPhi, sinV[1] * sinPhi, -cosPhi };
                p1 = screenEdgeMatricies.slice(i).t() * p1;
                p2 = screenEdgeMatricies.slice(i).t() * p2;

                // Do an atan2 to find out how far around this solution is
                float p1V = std::atan2(p1[1], p1[0]);
                float p2V = std::atan2(p2[1], p2[0]);

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

        // Sort the limits to make pairs
        std::sort(thetaLimits.begin(), thetaLimits.end());

        std::vector<std::pair<float, float>> output;
        for (uint i = 0; i < thetaLimits.size(); i += 2) {
            output.push_back(std::make_pair(thetaLimits[i], thetaLimits[i+1]));
        }

        return output;
    }

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , lut(0.1, 0.5) { // TODO make this based of the kinematics

        on<Configuration>("VisualMesh.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file VisualMesh.yaml
        });

        auto sphere = std::make_unique<MeshObjectRequest>();
        sphere->type = MeshObjectRequest::SPHERE;
        sphere->radius = 0.05;
        sphere->height = 0;
        sphere->intersections = 2;
        sphere->maxDistance = 10;
        sphere->hardLimit = false;

        auto cylinder = std::make_unique<MeshObjectRequest>();
        cylinder->type = MeshObjectRequest::CYLINDER;
        cylinder->radius = 0.05;
        cylinder->height = 1.15;
        cylinder->intersections = 2;
        cylinder->maxDistance = 10;
        cylinder->hardLimit = false;

        auto circle = std::make_unique<MeshObjectRequest>();
        circle->type = MeshObjectRequest::CIRCLE;
        circle->radius = 0.025;
        circle->height = 0;
        circle->intersections = 1;
        circle->maxDistance = 1;
        circle->hardLimit = true;

        emit<Scope::INITIALIZE>(sphere);
        emit<Scope::INITIALIZE>(cylinder);
        emit<Scope::INITIALIZE>(circle);

        on<Trigger<MeshObjectRequest>>().then([this] (const MeshObjectRequest& request) {
            lut.addShape(request);
        });

        on<Trigger<CameraParameters>>().then([this] (const CameraParameters& params) {

            // Work out the minimum angle we can jump
            double minimumAngleJump = 2 * std::atan(M_SQRT2 / params.focalLengthPixels);

            lut.setMinimumJump(minimumAngleJump);
        });

        on<Trigger<Image>, With<Sensors>, With<CameraParameters>, Single>().then([this] (const Image&, const Sensors& sensors, const CameraParameters& params) {

            // get field of view
            float fovX = params.FOV[0];
            float fovY = params.FOV[1];
            float focalLengthPixels = params.focalLengthPixels;

            // Camera height is z component of the transformation matrix
            float cameraHeight = sensors.orientationCamToGround(2, 3);

            // Remove yaw from our cameras rotation matrix and make it a float matrix
            arma::fmat33 camToGround = arma::conv_to<arma::fmat>::from(Rotation3D::createRotationZ(-sensors.orientationCamToGround.rotation().yaw()) * sensors.orientationCamToGround.rotation());

            // Our pixel offset value
            arma::fvec2 screenCentreOffset = arma::fvec2({ float(params.imageSizePixels[0] - 1), float(params.imageSizePixels[1] - 1) })  * 0.5;

            /***************************
             * Calculate image corners *
             ***************************/
            // Get the corners of the view port in cam space
            float yMax = std::tan(fovX * 0.5);
            float zMax = std::tan(fovY * 0.5);

            // Prenormalise our unit vectors
            float vecLength = 1 / std::sqrt(yMax*yMax + zMax*zMax + 1);
            float normYMax = yMax * vecLength;
            float normZMax = zMax * vecLength;

            arma::fmat::fixed<3,4> cornerPointsCam = {
                vecLength,  normYMax,  normZMax,
                vecLength, -normYMax,  normZMax,
                vecLength, -normYMax, -normZMax,
                vecLength,  normYMax, -normZMax
            };

            // Rotate the camera points into world space
            arma::fmat::fixed<3,4> cornerPointsWorld = camToGround * cornerPointsCam;

            /**************************************************
             * Calculate screen edge planes and corner angles *
             **************************************************/
            arma::fcube::fixed<3,3,4> screenEdgeMatricies;
            arma::fvec4 screenEdgeArcs;
            for(int i = 0; i < screenEdgeMatricies.n_slices; ++i) {
                // Make our screen edge rotation matrix
                screenEdgeMatricies.slice(i).col(0) = cornerPointsWorld.col(i);
                screenEdgeMatricies.slice(i).col(2) = arma::normalise(arma::cross(cornerPointsWorld.col(i), cornerPointsWorld.col((i + 1) % 4)));
                screenEdgeMatricies.slice(i).col(1) = arma::cross(screenEdgeMatricies.slice(i).col(2), screenEdgeMatricies.slice(i).col(0));

                // Find the end of our screen edge arc
                arma::fvec3 p = screenEdgeMatricies.slice(i).t() * cornerPointsWorld.col((i + 1) % 4);
                screenEdgeArcs[i] = std::atan2(p[1], p[0]);
            }

            /*************************
             * Calculate min/max phi *
             *************************/
            // Because our matrix has no yaw, we can directly extract cos and sin
            const float& cosRoll = camToGround(1, 1);
            const float& sinRoll = camToGround(2, 1);

            // Phi field of view comes from the vectors from the centre to the corner
            // There are two of these, so we choose the larger of the two
            float fovPhi = std::atan(std::max(std::abs(yMax * sinRoll + zMax * cosRoll)
                                             , std::abs(yMax * sinRoll - zMax * cosRoll)));
            float fovOffset = M_PI_2 - std::asin(camToGround(0, 2));

            float minPhi = fovOffset - fovPhi;
            float maxPhi = fovOffset + fovPhi;

            /******************************
             * Get our edges from our LUT *
             ******************************/
            lut.lookup(cameraHeight, minPhi, maxPhi
                , std::bind(thetaLimits, std::placeholders::_1, std::cref(screenEdgeMatricies), std::cref(screenEdgeArcs)));

            // /*************************************************************
            //  * Get our lookup table and loop through our phi/theta pairs *
            //  *************************************************************/
            // auto phiIterator = lut.getLUT(cameraHeight, minPhi, maxPhi);
            // std::vector<arma::fvec3> camPoints;

            // float lastPhi = std::numeric_limits<float>::min();
            // for(auto it = phiIterator.first; it != phiIterator.second; ++it) {

            //     const float& phi = it->first;
            //     const float& dTheta = it->second;

            //     // Check we jumped far enough
            //     if (phi - lastPhi >= minAngleJump) {

            //         lastPhi = phi;

            //         /*********************************************************
            //          * Calculate our min and max theta values for each plane *
            //          *********************************************************/
            //         float sinPhi = sin(phi);
            //         float cosPhi = cos(phi);

            //         std::vector<float> thetaLimits;
            //         thetaLimits.reserve(4);

            //         for(int i = 0; i < 4; ++i) {

            //             if (minPhi < phi && phi < maxPhi) {

            //                 // Get the unit vector that describes the normal to the screen plane
            //                 float& x = screenEdgeMatricies(0, 2, i);
            //                 float& y = screenEdgeMatricies(1, 2, i);
            //                 float& z = screenEdgeMatricies(2, 2, i);

            //                 // Solve our equation to find our theta intercepts
            //                 arma::fvec v = solveAcosThetaPlusBsinThetaEqualsC(sinPhi * x, sinPhi * y, cosPhi * z);

            //                 // We only care about the case with two solutions
            //                 if(v.n_elem == 2) {

            //                     arma::fvec2 cosV = arma::cos(v);
            //                     arma::fvec2 sinV = arma::sin(v);

            //                     // Calculate a unit vector to this solution and rotate it
            //                     // into the screen edge plane space (removing the z component)
            //                     arma::fvec3 p1 = { cosV[0] * sinPhi, sinV[0] * sinPhi, -cosPhi };
            //                     arma::fvec3 p2 = { cosV[1] * sinPhi, sinV[1] * sinPhi, -cosPhi };
            //                     p1 = screenEdgeMatricies.slice(i).t() * p1;
            //                     p2 = screenEdgeMatricies.slice(i).t() * p2;

            //                     // Do an atan2 to find out how far around this solution is
            //                     float p1V = atan2(p1[1], p1[0]);
            //                     float p2V = atan2(p2[1], p2[0]);

            //                     // Check solution 1 is between 0 and our other point
            //                     if (0 < p1V && p1V < screenEdgeArcs[i]) {
            //                         thetaLimits.push_back(v[0]);
            //                     }

            //                     // Check solution 2 is between 0 and our other point
            //                     if (0 < p2V && p2V < screenEdgeArcs[i]) {
            //                         thetaLimits.push_back(v[1]);
            //                     }
            //                 }
            //             }
            //         }

            //         // Sort the limits to make pairs
            //         std::sort(thetaLimits.begin(), thetaLimits.end());

            //         /***********************************
            //          * Loop through our theta segments *
            //          ***********************************/
            //         for (size_t i = 0; i < thetaLimits.size(); i += 2) {

            //             float minTheta = thetaLimits[i];
            //             const float& maxTheta = thetaLimits[i + 1];
            //             float thetaJump = std::min(dTheta, minAngleJump);

            //             // Theta must be a multiple of dtheta
            //             minTheta += minTheta > 0 ? std::fmod(std::abs(minTheta), thetaJump) : std::fmod(std::abs(2*M_PI - minTheta), thetaJump);

            //             // Loop through our valid theta range using delta theta
            //             for (float theta = minTheta; theta < maxTheta; theta += thetaJump) {


            //                 /*******************************************************************
            //                  * Add this phi/theta point to the list to project onto the screen *
            //                  *******************************************************************/
            //                 float cosTheta = std::cos(theta);
            //                 float sinTheta = std::sin(theta);

            //                 camPoints.push_back(camToGround.i() * arma::fvec3({ cosTheta * sinPhi, sinTheta * sinPhi, -cosPhi }));
            //             }
            //         }
            //     }
            // }

            // /***********************************************
            //  * Project our phi/theta pairs onto the screen *
            //  ***********************************************/
            // std::vector<arma::ivec2> screenPoints;
            // std::vector<std::pair<arma::ivec2, arma::ivec2>> helperPoints;

            // screenPoints.reserve(camPoints.size());
            // for(auto& point : camPoints) {

            //     arma::fvec2 floatPixel = screenCentreOffset - arma::fvec2({(focalLengthPixels * point[1] / point[0]), (focalLengthPixels * point[2] / point[0])});

            //     screenPoints.push_back(arma::ivec2({lround(floatPixel[0]), lround(floatPixel[1])}));

            //     helperPoints.push_back(std::make_pair(screenPoints.back(), screenPoints.back() + arma::ivec2({1,1})));
            // }

            // emit(drawVisionLines(std::move(helperPoints)));
        });
    }
}
}
