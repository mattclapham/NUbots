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
#include "message/support/Configuration.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace vision {
    
    using message::input::Image;
    using message::input::Sensors;
    using message::support::Configuration;

    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;

    struct MeshObjectRequest {
        enum Type {
            SPHERE,
            CIRCLE,
            CYLINDER
        };
    };

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("VisualMesh.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file VisualMesh.yaml
        });

        on<Trigger<MeshObjectRequest>>().then([] (const MeshObjectRequest&) {
            // Update list of shapes
            // Regnerate LUT
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

    std::vector<arma::vec3> VisualMesh::findCornerPoints(double xmax, double FOV_X, double FOV_Y) {
        double ymax = viewingAngleMax(xmax, FOV_X);
        double zmax = viewingAngleMax(xmax, FOV_Y);

        // calculates the corner points of the camera's field of view in camera space at a given x.
        std::vector<arma::vec3> corners;
        corners.push_back(arma::vec3({xmax, ymax, zmax}));
        corners.push_back(arma::vec3({xmax, -ymax, zmax}));
        corners.push_back(arma::vec3({xmax, ymax, -zmax}));
        corners.push_back(arma::vec3({xmax, -ymax, -zmax}));
        return corners;
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

    std::vector<double> VisualMesh::findThetaLimits(double phiDash, double theta, Transform3D camToGround, double FOV_X, double FOV_Y) {
        //Find Corner Points 
        std::vector<arma::vec3> cornerPointsCam = findCornerPoints(1, FOV_X, FOV_Y);
        //Transform them into world space
        std::vector<arma::vec3> cornerPointsWorld;
        for(auto& point : cornerPointsCam) {
            cornerPointsWorld.push_back(camToGround.transformPoint(point));
        }

        arma::vec3 cameraCentre = camToGround.submat(0,3,2,3); // position of camera centre
        //Find the four points of intersection of the field of view lines to the ground plane that form a quadrilateral
        // TODO maybe make arma::mat
        std::vector<arma::vec3> fieldOfViewQuadrilateral;
        for(auto& point : cornerPointsWorld) {
            fieldOfViewQuadrilateral.push_back(lineIntersectWithGroundPlane(cameraCentre, point));
        }

        // Find points of intersection of the circle defined by phi around the robot quadrilateral formed by the projected field of view.
        double radius = cameraCentre(2)*std::tan(phiDash);

        // TODO make into an array
        std::vector<arma::vec2> solutionsLine1 = lineIntersectWithCircle(arma::vec2({0,0}), radius, fieldOfViewQuadrilateral[0].rows(0,1), fieldOfViewQuadrilateral[1].rows(0,1));
        std::vector<arma::vec2> solutionsLine2 = lineIntersectWithCircle(arma::vec2({0,0}), radius, fieldOfViewQuadrilateral[1].rows(0,1), fieldOfViewQuadrilateral[2].rows(0,1));
        std::vector<arma::vec2> solutionsLine3 = lineIntersectWithCircle(arma::vec2({0,0}), radius, fieldOfViewQuadrilateral[2].rows(0,1), fieldOfViewQuadrilateral[3].rows(0,1));
        std::vector<arma::vec2> solutionsLine4 = lineIntersectWithCircle(arma::vec2({0,0}), radius, fieldOfViewQuadrilateral[3].rows(0,1), fieldOfViewQuadrilateral[0].rows(0,1));
        // Throw out solutions not within the line segments of the quadrilateral

        std::vector<arma::vec2> solutions;
        auto newSolutions = throwOutSolutionsNotInSegment(solutionsLine1, fieldOfViewQuadrilateral[0].rows(0,1), fieldOfViewQuadrilateral[1].rows(0,1));
        solutions.insert(newSolutions.begin(), newSolutions.end());
        newSolutions = throwOutSolutionsNotInSegment(solutionsLine2, fieldOfViewQuadrilateral[1].rows(0,1), fieldOfViewQuadrilateral[2].rows(0,1));
        solutions.insert(newSolutions.begin(), newSolutions.end());
        newSolutions = throwOutSolutionsNotInSegment(solutionsLine3, fieldOfViewQuadrilateral[2].rows(0,1), fieldOfViewQuadrilateral[3].rows(0,1));
        solutions.insert(newSolutions.begin(), newSolutions.end());
        newSolutions = throwOutSolutionsNotInSegment(solutionsLine4, fieldOfViewQuadrilateral[3].rows(0,1), fieldOfViewQuadrilateral[0].rows(0,1));
        solutions.insert(newSolutions.begin(), newSolutions.end());

        switch(solutions.size()) {
            case 2:
            case 4:
                std::vector<double> out();
                for(auto s : solutions) {
                    // TODO get theta for solution
                    double theta = std::acos(s[0] / radius);
                    out.push_back(theta);
                }

                std::sort(out.begin(), out.end());

                return out;

            default:
                return std::vector<double>();
        }
    }

}
}
