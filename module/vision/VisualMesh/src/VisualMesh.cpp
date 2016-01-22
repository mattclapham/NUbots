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
            // get camera orientation matrix and camera centre as the 4th column, without the 4th element
            Transform3D camToGround = sensors.orientationCamToGround; // 4x4 Transfomration matrix from Cam to Ground

            // get field of view
            float FOV_X = 1.0472;
            float FOV_Y = 0.785398;
            // Go nuts!
            double imageWidth = image.width;
            double imageHeight = image.height;
            double phiDash;

            //Find Angular Limits
            //Phi = ?????


            //For each phi, jump by theta within the limits, //NOT WRITTEN
            // to find the sample point and convert it to camera space //WRITTEN


            // chuck points out that are off the screen.


        });
    }

    std::vector<arma::vec2> VisualMesh::throwOutSolutionsNotInSegment(std::vector<arma::vec2> solutions, arma::vec2 a, arma::vec2 b) {
    std::vector<arma::vec2> segmentSolutions;
        for(auto& point : solutions) {
            bool checkIsTrue = checkPointInLineSegment(point, a, b);
            // if point is in segment, keep it
            if(checkIsTrue) {
                segmentSolutions.push_back(point);
            }
        }
        return segmentSolutions;
    }

    arma::mat::fixed<3,4> VisualMesh::findCornerPoints(double xmax, double FOV_X, double FOV_Y) {

        double ymax = viewingAngleMax(xmax, FOV_X);
        double zmax = viewingAngleMax(xmax, FOV_Y);

        // Return giving the column major value ordering
        return arma::mat::fixed<3,4>({
            xmax, ymax, zmax,
            xmax, -ymax, zmax,
            xmax, ymax, -zmax,
            xmax, -ymax, -zmax
        });
    }

    arma::vec3 VisualMesh::convertPhiAndThetaToCamSpace(double phiDash, double theta, Transform3D camToGround) {
        // phi and theta are converted to spherical coordinates in a space with the same
        // orientation as the world, but with origin at the camera position.
        // NOTE: the associated phi in spherical coords is given by -(pi/2 - phiDash), which simplifies the spherical coord conversion to:
        arma::vec3 sphericalCoords = {std::cos(theta)*std::sin(phiDash), std::sin(theta)*std::sin(phiDash), -std::cos(phiDash)};
        // To put in camera space multiply by the rotation matrix, use 0 for rotation
        arma::vec4 camSpacePoint_ = camToGround.i()*arma::join_cols(sphericalCoords, arma::vec({0}));
        // Discard the 4th element (= 0)
        arma::vec3 camSpacePoint = camSpacePoint.submat(0,0,2,0);
        return camSpacePoint;
        // Then use arma::vec2 projectCamSpaceToScreen(const arma::vec3& point, const double& camFocalLengthPixels) in Vision.h to convert to screen space.
    }


    std::vector<double> findPhiLimits(Transform3D camToGround, double FOV_X, double FOV_Y) {

        std::vector<arma::vec3> cornerPointsCam = findCornerPoints(1, FOV_X, FOV_Y);

        std::vector<arma::vec3> cornerPointsWorld;

        for(auto& point : cornerPointsCam) {
            cornerPointsWorld.push_back(arma::normalise(camToGround.transformVector(point)));
        }

        double phiMin = std::numeric_limits<double>::max();
        double phiMax = std::numeric_limits<double>::min();

        for(auto p : cornerPointsWorld) {
            double phi = acos(p(2));
            phiMin = std::min(phiMin, phi);
            phiMin = std::min(phiMin, phi);
        }

        return std::vector(phiMin, phiMax);
    }

    std::vector<double> VisualMesh::findThetaLimits(double phiDash, Transform3D camToGround, double FOV_X, double FOV_Y) {
        //Find Corner Points
        arma::mat::fixed<3,4> cornerPointsCam = findCornerPoints(1, FOV_X, FOV_Y);

        // Get the radius of the ground circle
        double circleRadius = cameraHeight * std::tan(phiDash);

        // Rotate our camera vectors to world rotation
        arma::mat::fixed<3,4> cornerPointsWorld = camToGround.rotation() * cornerPointsCam; // 3x4

        // Intersect with the ground plane and ignore the z component
        arma::mat::fixed<2,4> groundPoints = (cornerPointsWorld * (-cameraHeight / cornerPointsWorld.row(2).t())).rows(0,1);
        // Get direction vectors from each corner point to the next corner point
        arma::mat::fixed<2,4> groundDirections = groundPoints - groundPoints.cols({1,2,3,0});

        // Get all our circle intersection values
        arma::mat::fixed<2,4> c1 = groundPoints * groundDirections;
        arma::vec4            c2 = the element wise norm of groundPoints;
        disc = arma::square(c1) - arma::square(c2) + (circleRadius * circleRadius);

        // Get the indicies of the vectors that intersected twice
        // we don't care about once as it won't impact our ranges
        arma::uvec intersected = arma::find(disc > 0);

        // Get our actual intersection points
        arma::mat intersectionPoints(2, intersected.n_cols * 2);
        intersectionPoints.cols(intersected * 2)       = groundPoints.cols(intersected) + c1.cols(intersected) + arma::sqrt(disc.cols(intersected));
        intersectionPoints.cols((intersected + 1) * 2) = groundPoints.cols(intersected) + c1.cols(intersected) - arma::sqrt(disc.cols(intersected));

        // Remove the points that are not in the segments
        // TODO TODO TODO TODO TODO

        // Convert the solutions to their theta value
        // TODO TODO TODO TODO TODO

        // Sort the theta values and return

    }

}
}
