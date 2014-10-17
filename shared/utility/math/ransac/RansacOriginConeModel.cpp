/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "RansacOriginConeModel.h"

namespace utility {
namespace math {
namespace ransac {

    bool RansacOriginConeModel::regenerate(const std::vector<DataPoint>& points) {

        if (points.size() == REQUIRED_POINTS
            && !arma::all(points[0] == points[1])
            && !arma::all(points[0] == points[2])
            && !arma::all(points[1] == points[2])) {

            // Find the unit vector such that a.d = b.d = c.d = 1

            arma::mat33 matrix;
            matrix.row(0) = points[0].t();
            matrix.row(1) = points[1].t();
            matrix.row(2) = points[2].t();

            // Solve for our centre
            centre = arma::normalise(arma::solve(matrix, arma::vec3({1, 1, 1})));

            // Solve for our radius
            radius = std::acos(arma::dot(points[0], centre));

            return true;
        }

        else {
            return false;
        }
    }

    double RansacOriginConeModel::calculateError(const DataPoint& p) const {

        double error = radius - std::acos(arma::dot(centre, p));
        return error * error;
    }

}
}
}
