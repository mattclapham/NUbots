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

#include "RansacOriginPlaneModel.h"

namespace utility {
namespace math {
namespace ransac {

  bool RansacOriginPlaneModel::regenerate(const std::vector<DataPoint>& points) {

        if(points.size() == REQUIRED_POINTS && !arma::all(points[0] == points[1])) {
            normal = arma::normalise(arma::cross(points[0], points[1]));
            return true;
        }

        else {
            return false;
        }
    }

    double RansacOriginPlaneModel::calculateError(const DataPoint& p) const {
        double cosD = arma::dot(normal, p);
        return cosD * cosD;
    }

}
}
}
