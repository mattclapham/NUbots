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

#include "VisualMeshLUT.h"
#include "utility/math/matrix/Transform3D.h"

namespace module {
namespace vision {

    class VisualMesh : public NUClear::Reactor {

    private:
        VisualMeshLUT lut;

		// calculates the length of the opposite side of the triangle which gives the maximum value of the other coord (x or z)
		// in the FOV plane for a given x, in camera space.
    	double viewingAngleMax(double xmax, double FOV) {
			return xmax*std::tan(FOV/2);
    	}

		// check if point is in line segment ab. WARNING does not consider point == segment point as in segment
		bool checkPointInLineSegment(arma::vec2 point, arma::vec2 a, arma::vec2 b) {
		    double xmin = std::min(a(0), b(0));
		    double xmax = std::max(a(0), b(0));

		    double ymin = std::min(a(1), b(1));
		    double ymax = std::max(a(1), b(1));

		    double xdiff = xmax - xmin;
		    double ydiff = ymax - ymin;

		    if(xdiff < ydiff) {
		        return ymin < point(1) && point(1) < ymax;
		    } else {
		        return xmin < point(0) && point(0) < xmax;
			}
		}

		std::vector<arma::vec2> throwOutSolutionsNotInSegment(std::vector<arma::vec2> solutions, arma::vec2 a, arma::vec2 b);
		arma::mat::fixed<3,4> findCornerPoints(double xmax, double FOV_X, double FOV_Y);
		arma::vec3 convertPhiAndThetaToCamSpace(double phiDash, double theta, utility::math::matrix::Transform3D camToGround);
        std::pair<double, double> findPhiLimits(utility::math::matrix::Transform3D camToGround, double FOV_X, double FOV_Y);
        std::vector<double> findThetaLimits(double phiDash, utility::math::matrix::Transform3D camToGround, double FOV_X, double FOV_Y);

    public:
        /// @brief Called by the powerplant to build and setup the VisualMesh reactor.
        explicit VisualMesh(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_VISION_VISUALMESH_H

/*
public static vector3 Intersect_RS(Ray3D R, Sphere3D S)
        {
            float t, QFb, QFc, disc;

            #region #region: No Vectors
            */
            /*
            float a, b, c, r, m, n, p, q, u, v;
            a = S.C.x; b = S.C.y; c = S.C.z; r = S.r;
            m = R.P0.x; n = R.D.x; p = R.P0.y; q = R.D.y; u = R.P0.z; v = R.D.z;
            QFb = 2 * (n * (m - a) + q * (p - b) + v * (u - c));
            QFc = (m - a) * (m - a) + (p - b) * (p - b) + (v - c) * (v - c) - r * r;
            */
/*
            #endregion

            vector3 P0_SC = R.P0 - S.c;
            QFb = 2 * vector3.dot(R.D, P0_SC);
            QFc = vector3.dot(P0_SC, P0_SC) - S.r * S.r;

            if ((disc = QFb * QFb - 4 * QFc) < 0) { return null; }
            disc = (float)Math.Sqrt(disc);
            if ((t = -0.5f * (QFb + disc)) > 0) { return R.P0 + R.D * t; }
            if ((t += disc) > 0) { return R.P0 + R.D * t; }
            return null;
        }
*/

/*
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
*/

/*
		// given two points find the coefficient of x and value of b for the form y = mx + b.
		arma::vec2 cartesianLineEquation(arma::vec2 point1, arma::vec2 point2) {
			double gradient = (point2(1) - point1(1))/(point2(0) - point1(0));
			double b = point1(1) - gradient*point1(0);
			return arma::vec2({gradient, b});
		}

*/