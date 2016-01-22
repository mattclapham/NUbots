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
#include "message/input/Sensors.h"
#include "message/vision/MeshObjectRequest.h"
#include "message/support/Configuration.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"

#include "utility/motion/RobotModels.h"

namespace module {
namespace vision {

    using message::input::Image;
    using message::input::Sensors;
    using message::vision::MeshObjectRequest;
    using message::support::Configuration;

    using utility::motion::kinematics::DarwinModel;

    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , lut(0.2, 0.5) { // TODO make this based of the kinematics

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

        on<Trigger<Image>, With<Sensors>>().then([this] (const Image& image, const Sensors& sensors) {

            // get field of view
            float FOV_X = 1.0472;
            float FOV_Y = 0.785398;

            // Camera height is z component of the transformation matrix
            double cameraHeight = sensors.orientationCamToGround(2, 3);

            /***************************
             * Calculate image corners *
             ***************************/
            // Get the corners of the view port in cam space
            double xmax = std::tan(FOV_X/2);
            double ymax = std::tan(FOV_Y/2);
            arma::mat::fixed<3,4> camCornerPoints = {
                1,  ymax,  zmax,
                1, -ymax,  zmax,
                1,  ymax, -zmax,
                1, -ymax, -zmax
            };

            // Rotate the camera points into world space
            arma::mat::fixed<3,4> worldCornerPoints = sensors.orientationCamToGround.rotation() * camCornerPoints;


            /*************************
             * Calculate min/max phi *
             *************************/
            // Get the cosv value
            arma::vec4 phiCosV = arma::acos(cornerPointsWorld.row(2));

            // Return the minimum and maximum values
            double minPhi = phiCosV.min();
            double maxPhi = phiCosV.max();


            /***********************************
             * Calculate ground line equations *
             ***********************************/
            // Intersect our corner points with the ground plane and drop the z component
            arma::mat::fixed<2,4> groundPoints = arma::mat(cornerPointsWorld * (-cameraHeight / cornerPointsWorld.row(2).t())).rows(0,1);
            // Get direction vectors from each corner point to the next corner point
            arma::mat::fixed<2,4> groundDirections = groundPoints - groundPoints.cols(arma::uvec({1,2,3,0}));


            /**************************************************
             * Solve as much of the circle equation as we can *
             **************************************************/
            // Get all our circle intersection values
            arma::vec4 circleEq1 = groundPoints * groundDirections;
            arma::vec4 circleEq2;
            for(size_t i = 0; i < circleEq2.n_cols; ++i) {
                circleEq2[i] = arma::norm(groundPoints.col(i));
            }

            // Calculate our apart from the radius
            arma::vec4 partialDiscriminant = arma::square(c1) - arma::square(c2);

            // Calculate our solution bar the discriminant
            arma::mat intersectionPoints = groundPoints + c1;


            /*************************************************************
             * Get our lookup table and loop through our phi/theta pairs *
             *************************************************************/
            auto phiIterator = getLUT(cameraHeight, minPhi, maxPhi);

            for(auto it = phiIterator.first; it != phiIterator.second; ++it) {

                const double& phi = it->first;
                const double& dTheta = it->second;

                /**************************************************
                 * Calculate the remainder of the circle equation *
                 **************************************************/

                // Calculate the radius of the circle
                double circleRadius = cameraHeight * std::tan(phi);

                // Calculate the actual discriminant values
                arma::vec4 discriminants = partialDiscriminant + (circleRadius * circleRadius);

                // Find the discriminants that will yeild two solutions
                arma::uvec indicies = arma::find(discriminants > 0);

                // Find our intersection points
                // Square root the relevant discriminants
                arma::vec vals = arma::sqrt(discriminants.cols(indicies));
                intersectionPoints + vals;
                intersectionPoints - vals;

                /****************************************************************************************
                 * Eliminate solutions that are not on the screen and get remaining min/max theta pairs *
                 ****************************************************************************************/

                // TODO eliminate intersection points that are not on the screen and get min/max theta pairs
                std::vector<double> thetaPairs;

                /***********************************
                 * Loop through our theta segments *
                 ***********************************/
                for (size_t i = 0; i < thetaPairs.size() / 2; i += 2) {
                    const double& minTheta = thetaPairs[i];
                    const double& maxTheta = thetaPairs[i + 1];

                    for (double theta = minTheta; theta < maxTheta; theta += dTheta) {

                        /********************************
                         * Add this phi/theta to a list *
                         ********************************/

                        // TODO here we have phi and theta and can calculate a screen point and add it to a list

                    }
                }
            }

            /***********************************************
             * Project our phi/theta pairs onto the screen *
             ***********************************************/

            // TODO project the phi/theta pais onto the screen
            for(auto& point : phiThetaPoints) {

                coords = aofijeafijse{cos(theta)*sin(phi), sin(theta)*sin(phi), -cos(phi)}
                camToGround.rotation().i() * coords;
                // We are now in cam space?

            }
        });
    }

}
}