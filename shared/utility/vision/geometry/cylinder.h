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
#ifndef UTILITY_MATH_VISION_H
#define UTILITY_MATH_VISION_H

#include <cmath>
#include <armadillo>

namespace utility {
namespace vision {
namespace geometry {
    //NOTE: Vertical indicates upright poles, GroundPlane indicates poles lying on the groundplane

    inline double arcSizeFromTopRayVertical(const arma::vec3& ray, const arma::vec2& objectSize, const double& cameraHeight) {
        //returns the arcsize for a sphere this ray grazes the top of
        double angle = acos(ray[2]);
        
        //find horizontal distance to the pole
        horizontalDist = abs((cameraHeight-objectSize[1])/tan(angle));
        
        //find the sum of angles to get the predicted arcsize
        return atan2(cameraHeight,horizontalDist) + atan2(objectSize[1]-cameraHeight,horizontalDist);
    }
    
    inline double arcSizeFromBaseRayVertical(const arma::vec3& ray, const double& objectSize, const double& cameraHeight) {
        //returns the arcsize for a sphere this ray grazes the top of
        double angle = acos(ray[2]);
        
        //find horizontal distance to the pole
        horizontalDist = abs((cameraHeight)/tan(angle));
        
        //find the sum of angles to get the predicted arcsize
        return atan2(cameraHeight,horizontalDist) + atan2(objectSize[1]-cameraHeight,horizontalDist);
        
    }
    
    inline double arcSizeFromTopRayGroundPlane(const arma::vec3& ray, const arma::vec2& objectSize, const double& cameraHeight, const double& orientation) {
        //XXX: unimplemented
    }
    
    inline double arcSizeFromBaseRayGroundPlane(const arma::vec3& ray, const double& objectSize, const double& cameraHeight, const double& orientation) {
        //XXX: unimplemented
    }
    
    inline double distanceFromCentreRay(const arma::vec3& ray, const double& objectSize, const double& cameraHeight) {
        //returns the distance along the ray of the object using elevation and angle
        //Object size is {width,height}
        return abs((cameraHeight - objectSize[1]/2)/ray[2]);
    }
    
    inline double distanceFromArcWidth(const arma::vec3& ray, const arma::vec2& objectSize, const double& arcRadius) {
        //returns the distance along the ray of the object using arcsize
        //Object size is {width,height}
        //NOTE: this is spherical, may be a few % off due to cylindrical object
        return abs(objectSize/sin(arcRadius[0]));
    }
    
    inline double distanceFromArcHeightVertical(const arma::vec3& ray, const arma::vec2& objectSize, const double& arcRadius) {
        //returns the distance along the ray of the object using arcsize
        //Object size is {width,height}
        
        //XXX: copy the crazy method from the old code
        
        return 0;
    }
    
    inline double distanceFromArcHeightGroundPlane(const arma::vec3& ray, const arma::vec2& objectSize, const double& arcRadius) {
        //Assuming the cylinder is horizontal in the ground plane, we can do something like this
        //Object size is {width,height}
        //XXX: need a new crazy method
        
        return objectSize/sin(arcRadius[0]);
    }
    
}
}
}

#endif
