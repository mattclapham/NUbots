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

arma::mat intersect(const ParametricLine<2>& line, const Circle& circle) {
    // https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // x = position + t*direction is line sub into circle: ||x - c||^2 = r^2
    // solve for t
    double calc0 = -arma::dot(line.position - circle.centre, line.direction);
    double calc1 = arma::norm(line.position - circle.centre);
    double disc = (calc0 * calc0 - calc1 * calc1 + circle.radius * circle.radius);

    // sub t into line eq to get the points of intersection
    if (disc > 0) {
        result = arma::mat(2,2);
        result.col(0) = line.position + (calc0 + std::sqrt(disc)) * line.direction;
        result.col(1) = line.position + (calc0 - std::sqrt(disc)) * line.direction;
        return result;
    }
    else if (disc == 0) {
        return (line.position + calc0 * line.direction)
    }
    else {
        return arma::mat();
    }
}

template <size_t Dimension>
arma::mat intersect(const ParametricLine<Dimension>& line, const Plane<Dimension>& plane) {
    double dotN = arma::dot(line.direction, plane.normal);

    // does not meet;
    if(dotN == 0) {
        return arma::mat();
    }

    double tIntersection = arma::dot(line.direction, plane.normal);

    arma::dot(plane.point - line.point, plane.normal) / dotN;
    return tIntersection * line.direction + line.point;
}

arma::mat intersect(const Circle& circle, const ParametricLine<2>& line) {
    intersect(line, circle);
}


arma::mat intersect(const Line& line, const Line& line) {

}

template <size_t Dimensions>
arma::mat operator& (const ParametricLine<Dimensions>& line, const ParametricLine<Dimensions>& line) {

}
