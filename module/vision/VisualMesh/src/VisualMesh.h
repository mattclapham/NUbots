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

#ifndef MODULE_VISION_VISUALMESH_H
#define MODULE_VISION_VISUALMESH_H

#include <nuclear>
#include <armadillo>

namespace module {
namespace vision {

    class VisualMesh : public NUClear::Reactor {
    
    private:

		// calculates the length of the opposite side of the triangle which gives the maximum value of the other coord (x or z)
		// in the FOV plane for a given x, in camera space.
    	double viewingAngleMax(double xmax, double FOV) {
			return xmax*std::tan(FOV/2);
    	}

		// line = position + t*direction, intersects with the plane z = 0 when t = -p_z/d_z.
		arma::vec3 lineIntersectionWithGroundPlane(arma::vec3 positionVector, arma::vec3 directionVector) {
			arma::vec3 pointOfIntersection = positionVector - (positionVector(2)/directionVector(2))*directionVector;
			return pointOfIntersection;
		}

		// given two points find the coefficient of x and value of b for the form y = mx + b.
		arma::vec2 cartesianLineEquation(arma::vec2 point1, arma::vec2 point2) {
			double gradient = (point2(1) - point1(1))/(point2(0) - point1(0));
			double b = point1(1) - gradient*point1(0);
			return arma::vec2({gradient, b});
		}

		// http://www.math-only-math.com/a-cos-theta-plus-b-sin-theta-equals-c.html
		arma::vec solveAcosThetaPlusBsinThetaEqualsC(double a, double b, double c) {
			double r = std::sqrt(a*a + b*b);
			if(std::abs(c) <= r) {
				double alpha = std::atan(b/a);
				double beta = std::acos(c/r);
				if(beta == 0) {
					return arma::vec({ beta + alpha });
				} else {
					return arma::vec({ alpha + beta, alpha - beta });
				}
			} else {
				return arma::vec();
			}
		}

		std::vector<arma::vec3> findCornerPoints(double xmax, double FOV_X, double FOV_Y);
		arma::vec3 convertPhiAndThetaToCamSpace(double phiDash, double theta, arma::mat44 camToGround);

    public:
        /// @brief Called by the powerplant to build and setup the VisualMesh reactor.
        explicit VisualMesh(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_VISION_VISUALMESH_H
