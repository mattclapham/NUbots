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
#ifndef UTILITY_VISION_SPHERE_H
#define UTILITY_VISION_SPHERE_H

#include <cmath>
#include <armadillo>
#include <nuclear>
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "utility/math/matrix.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/ParametricLine.h"

namespace utility {
namespace vision {
namespace geometry {
namespace sphere {

    inline double arcSizeFromTopRay(const arma::vec3& ray, const double& objectRadius, const double& cameraHeight) {
        //returns the arcsize for a sphere this ray grazes the top of
        
        //this is the far ground intersection point (in terms of distance along the ray).
        const double d = cameraHeight / ray[2]; 
        
        //get the radians inclination from straight down.
        const double inclination = acos(ray[2]);
        
        //this is the distance along the ray from the sphere intersection to the far ground point.
        const double d2 = atan(M_PI/4 + inclination/2)*objectRadius;
        
        //this is the distance along the ray from the origin to the sphere intersection.
        const double d1 = d - d2;
        
        //using bisection of a kite, this is the arc angle of the sphere.
        return tan(objectRadius/d1)*2;
    }
    
    inline double arcSizeFromBaseRay(const arma::vec3& ray, const double& objectRadius, const double& cameraHeight) {
        //returns the arcsize for a sphere this ray grazes the bottom of
        
        //this is the ground intersection point (in terms of distance along the ray).
        const double d = cameraHeight / ray[2]; 
        
        //get the radians inclination from straight down.
        const double inclination = acos(ray[2]);
        
        //this is the distance along the ray from the sphere intersection to the far ground point.
        //NOTE: this is different to the above formula for top rays!
        const double d2 = objectRadius / atan(M_PI/4 + inclination/2);
        
        //this is the distance along the ray from the origin to the sphere intersection.
        const double d1 = d - d2;
        
        //using bisection of a kite, this is the arc angle of the sphere.
        return tan(objectRadius/d1)*2;
        
    }
    
    inline double distanceFromCentreRay(const arma::vec3& ray, const double& objectRadius, const double& cameraHeight) {
        //returns the distance along the ray of the object using elevation and angle
        return abs( (cameraHeight/objectRadius) / ray[2] );
    }
    
    inline double distanceFromArcSize(const double& arcRadius, const double& objectRadius) {
        //returns the distance along the ray of the object using arcsize
        return abs(objectRadius/sin(arcRadius));
    }
}
}
}
}

#endif
