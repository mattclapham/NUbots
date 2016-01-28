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
#include "message/motion/ServoTarget.h"

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
    using message::motion::ServoTarget;

    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;

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

        on<Trigger<Sensors>, Single>().then([this] (const Sensors& sensors) {



            // get field of view
            // 40 degrees
            float FOV_X = 1.0472;
            // 30 degrees
            float FOV_Y = 0.785398;

            // use resolution and FOV
            int camFocalLengthPixels = 10;
            // Camera height is z component of the transformation matrix
            double cameraHeight = sensors.orientationCamToGround(2, 3);

            /***************************
             * Calculate image corners *
             ***************************/
            
            // Get the corners of the view port in cam space
            double ymax = std::tan(FOV_X / 2);
            double zmax = std::tan(FOV_Y / 2);
            arma::mat::fixed<3,4> cornerPointsCam = {
                1,  ymax,  zmax,
                1, -ymax,  zmax,
                1, -ymax, -zmax,
                1,  ymax, -zmax
            };

            cornerPointsCam /= arma::norm(cornerPointsCam.col(0));

            // Rotate the camera points into world space

            arma::mat::fixed<3,4> cornerPointsWorld = sensors.orientationCamToGround.rotation() * cornerPointsCam;

            std::cout << "cornerPointsCam = " << cornerPointsCam << std::endl;
            std::cout << "sensors.orientationCamToGround.rotation() = " << sensors.orientationCamToGround.rotation() << std::endl;
            std::cout << "cornerPointsWorld = " << cornerPointsWorld << std::endl;

            /*************************
             * Calculate min/max phi *
             *************************/

            // TODO THIS DOESN'T WORK WHEN CROSSING VERTICAL AS Z COMPONENTS ARE THE SAME

            // Get the cosv value
            arma::rowvec4 phiCosV = arma::acos(-cornerPointsWorld.row(2));
            // Return the minimum and maximum values
            double minPhi = phiCosV.min();
            double maxPhi = phiCosV.max();
            //std::cout << "phiCosV = " << phiCosV << std::endl; 
            //std::cout << "minPhi = " << minPhi << std::endl; 
            //std::cout << "maxPhi = " << maxPhi << std::endl; 

            /***********************************
             * Calculate ground line equations *
             ***********************************/
            
            // Intersect our corner points with the ground plane and drop the z component
            // transform to find intersection points of corner lines and ground plane
            arma::vec4 zComponents = cornerPointsWorld.row(2).t();
            // invert each element and multiply by the camera height
            arma::mat44 groundTransform = arma::diagmat(-cameraHeight/zComponents);
            // only multiply the first two rows of corner points because we only want the x, y components in the end
            arma::mat::fixed<2,4> groundPoints = arma::mat(cornerPointsWorld.rows(0,1) * groundTransform);
            // Get direction vectors from each corner point to the next corner point
            arma::mat::fixed<2,4> groundDirections = groundPoints - groundPoints.cols(arma::uvec({1,2,3,0}));

            std::cout << "Ground Points" << std::endl;
            std::cout << groundPoints << std::endl;

            // arma::rowvec4 phiCosVDEBUG;

            // for(size_t i = 0; i < 4; ++i) {
            //     phiCosVDEBUG(i) = std::atan(std::sqrt(groundPoints(0, i) * groundPoints(0, i) + groundPoints(1, i) * groundPoints(1, i)) / cameraHeight);
            // }
            // //std::cout << "phiCosV = " << phiCosV << std::endl;
            // //std::cout << "phiCosVDEBUG = " << phiCosVDEBUG << std::endl;
            // double minPhi = phiCosVDEBUG.min();
            // double maxPhi = phiCosVDEBUG.max();
            //std::cout << "minPhi = " << minPhi << std::endl; 
            //std::cout << "maxPhi = " << maxPhi << std::endl; 



            /***************************************************************************
             * Solve as much of the circle intersection with screen equation as we can *
             ***************************************************************************/
            /*
            // https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
                std::vector<arma::vec2> rayIntersectWithSphere(arma::vec2 centre, double radius, arma::vec2 point_a, arma::vec2 point_b) {
                arma::vec2 direction = arma::normalise(point_b - point_a);
                arma::vec2 position = point_a;
                // x = position + t*direction is ray sub into sphere: ||x - c||^2 = r^2
                // solve for t
                double calc0 = -arma::dot(position - centre, direction);
                double calc1 = arma::norm(position - centre);
                double disc = calc0*calc0 - calc1*calc1 + radius*radius;
                t = calc0 +- sqrt(disc);
                POI = position + t*direction;
            */

            arma::vec4 circleEq1 = arma::diagvec(groundPoints.t() * groundDirections);
            arma::vec4 circleEq2;
            for(size_t i = 0; i < circleEq2.n_cols; ++i) {
                circleEq2[i] = arma::norm(groundPoints.col(i));
            }

            // Calculate the constant part of the discriminate
            arma::vec4 partialDiscriminant = arma::square(circleEq1) - arma::square(circleEq2);

            // Calculate our solution bar the discriminant
            // intersection point = groundPoint + t*groundDirection;
            //                    = groundPoint + (-circleEq1 +- sqrt(fulldisc))*groundDirection;
            //                    = groundPoint - circleEq1*groundDirection +- sqrt(fulldisc)*groundDirection;
            //                    = partialIntersectionPoints +- sqrt(fulldisc)*groundDirection; 
            arma::mat::fixed<2,4> partialIntersectionPoints = groundPoints;
            for(size_t i = 0; i < partialIntersectionPoints.n_cols; ++i) {
                partialIntersectionPoints.col(i) = partialIntersectionPoints.col(i) - circleEq1(i) * groundDirections.col(i);
            }

            /*************************************************************
             * Get our lookup table and loop through our phi/theta pairs *
             *************************************************************/

            auto phiIterator = lut.getLUT(cameraHeight, minPhi, maxPhi);
            std::vector<std::pair<double, double>> phiThetaPoints;

            std::cout << std::distance(phiIterator.first, phiIterator.second) << std::endl;

            for(auto it = phiIterator.first; it != phiIterator.second; ++it) {

                const double& phi = it->first;
                const double& dTheta = it->second;

                //std::cout << "phi = " << phi << std::endl; 
                /******************************************************************************************************************
                 * Calculate the remainder of the circle intersection equation and eliminate solutions that are not on the screen *
                 ******************************************************************************************************************/

                // Calculate the radius of the circle
                double circleRadius = cameraHeight * std::tan(phi);
                arma::vec4 discriminants = partialDiscriminant + circleRadius * circleRadius;

                // Find the discriminants that will yield two solutions
                arma::uvec indices = arma::find(discriminants > 0);

                // Square root the relevant discriminants
                arma::vec vals = arma::sqrt(discriminants.rows(indices));
      
                // Eliminate values which give solutions outside of the line segments, that is t = -circleEq1(i) +- sqrt(fulldisc) needs to be between 0 and 1
                arma::uvec plusIndices = arma::find(0 <= -circleEq1.rows(indices) + vals &&  -circleEq1.rows(indices) + vals <= 1);
                arma::uvec minusIndices = arma::find(0 <= -circleEq1.rows(indices) - vals &&  -circleEq1.rows(indices) - vals <= 1);
                arma::vec plusVals = vals.rows(plusIndices);
                arma::vec minusVals = vals.rows(minusIndices);
                arma::mat importantPlusIntersectionPoints = partialIntersectionPoints.cols(plusIndices);
                arma::mat importantPlusGroundDirections = groundDirections.cols(plusIndices);
                arma::mat importantMinusIntersectionPoints = partialIntersectionPoints.cols(minusIndices);
                arma::mat importantMinusGroundDirections = groundDirections.cols(minusIndices);

                // Find Intersection points = partialIntersectionPoints +- sqrt(fulldisc)*groundDirection;
                arma::mat intersectionPoints;
                intersectionPoints.set_size(2, plusIndices.n_rows + minusIndices.n_rows);
                for(size_t i = 0; i < plusIndices.n_rows; ++i) {
                    intersectionPoints.col(i) = importantPlusIntersectionPoints.col(i) + plusVals(i)*importantPlusGroundDirections.col(i);
                }
                for(size_t i = plusIndices.n_rows; i < plusIndices.n_rows + minusIndices.n_rows; ++i) {
                    intersectionPoints.col(i) = importantMinusIntersectionPoints.col(i) - minusVals(i)*importantMinusGroundDirections.col(i);
                }

                std::cout << "Phi = " << phi << std::endl;
                std::cout << "Radius = " << circleRadius << std::endl;

                std::cout << "IntersectionPoints: " << intersectionPoints << " Circle: " << circleRadius << std::endl;

                /***********************************************************************************************************************************
                 * Calculate theta values for intersection points  and sort them to make pairs that represent segments of the circle on the screen *
                 ***********************************************************************************************************************************/

                std::vector<double> thetaPairs;
                if (intersectionPoints.n_cols % 2 == 0) {
                    for(size_t i = 0; i < intersectionPoints.n_cols; ++i) {

                        //std::cout << "Intersection: " << intersectionPoints(0, i) << " Circle: " << circleRadius << std::endl;
                        
                        double theta = std::acos(intersectionPoints(0, i) / circleRadius);
                        //std::cout << "phi:" << phi << " theta = " << theta << std::endl;
                        thetaPairs.push_back(theta);
                    }
                    std::sort(thetaPairs.begin(), thetaPairs.end());
                }


                /***********************************
                 * Loop through our theta segments *
                 ***********************************/
                
                for (size_t i = 0; i < thetaPairs.size() / 2; i += 2) {
                    const double& minTheta = thetaPairs[i];
                    const double& maxTheta = thetaPairs[i + 1];

                    for (double theta = minTheta; theta < maxTheta; theta += dTheta) {

                        /************************************************************************
                         * Add this phi/theta sample point to a list to project onto the screen *
                         ************************************************************************/
                        
                        phiThetaPoints.push_back(std::make_pair(phi, theta));

                    }
                }
            }

            /***********************************************
             * Project our phi/theta pairs onto the screen *
             ***********************************************/

            std::vector<arma::vec2> screenPoints;

            for(auto& point : phiThetaPoints) {
                // phi and theta are converted to spherical coordinates in a space with the same orientation as the world, but with origin at the camera position. 
                // The associated phi in spherical coords is given by -(pi/2 - phi), which simplifies the spherical coordinate conversion to
                arma::vec3 sphericalCoords = { std::cos(point.second)*std::sin(point.first), std::sin(point.second)*std::sin(point.first), -std::cos(point.first) };
                // To put in camera space multiply by the ground to camera rotation matrix
                arma::vec3 camSpacePoint = sensors.orientationCamToGround.rotation().i() * sphericalCoords;
                // Project camera space to screen space by
                arma::vec2 screenSpacePoint = arma::vec2({camFocalLengthPixels * camSpacePoint[1] / camSpacePoint[0], camFocalLengthPixels * camSpacePoint[2] / camSpacePoint[0]});
                screenPoints.push_back(screenSpacePoint);

                //std::cout << screenPoints.back() << std::endl;
            }
        });
    }

}
}