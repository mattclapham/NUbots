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
#include <nuclear>
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "utility/math/matrix.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/ParametricLine.h"

namespace utility {
namespace vision {
namespace geometry {

    inline arma::imat snapToScreen(const arma::imat& rayPositions, const arma::ivec& rayLength, const Image& image) {
        arma::mat scales(rayPositions.n_rows,2);
        if (radial) {
        
        
        } else if (rectilinear) {
            scales.col(0) = ;
        
        }
    }
    
    inline arma::imat trimToScreen(const arma::imat& pixels, const Image& image) {
        //remove any pixel references not within the screen area
        arma::imat result = pixels(arma::find(pixels.col(1) >= 0));
        result = pixels(arma::find(pixels.col(1) < image.width));
        result = pixels(arma::find(pixels.col(0) >= 0));
        result = pixels(arma::find(pixels.col(0) < image.height));
        return std::move(result);
    }
    
    inline arma::imat bulkRay2Pixel(const arma::mat& rays, const Image& image) {
        //convert camera rays to pixels
        arma::mat result;
        
        if (radial) {
            arma::vec2 imageCenter = arma::vec2({image.lensParams[0],
                                                     image.lensParams[1]});
            double pixelPitch = image.lensParams[2];
            
            result = rays.cols(0,1);
            
            arma::vec rads = arma::acos(rays.col(2));
            
            result /= arma::repmat(arma::sqrt(arma::sum(arma::square(result),1))*pixelPitch,2,1);
            result *= arma::repmat(rads,2,1);
            result += arma::repmat(imageCenter,1,rays.n_rows);
        
        } else if (rectilinear) {
            
            arma::vec2 imageCenter = arma::vec2({image.height/2,
                                                 image.width/2});
            
            double focalLength = image.lensParams[0];
            
            result = rays.cols(0,1);
            
            result *= arma::repmat(focalLength/rays.col(2),2,1);
            
            result += arma::repmat(imageCenter,1,rays.n_rows);
        }
        
        
        return arma::conv_to<arma::imat>::from(arma::round(result));
    }
    
    inline arma::mat bulkPixel2Ray(const arma::imat& pixels, const Image& image) {
        //convert a matrix of rows of 2d pixels into spherical camera rays
        
        arma::mat result(pixels.n_rows,3);
        if (radial) {
            //radial lens conversion
            arma::vec2 imageCenter = arma::vec2({image.lensParams[0],
                                                 image.lensParams[1]});
            double pixelPitch = image.lensParams[2];
            
            //center the pixels
            const arma::mat px = arma::conv_to<arma::mat>::from(pixels) - 
                                 arma::repmat(imageCenter.t(),1,pixels.n_rows) *
                                 pixelPitch;
            
            //get all the radian values
            arma::vec rads = arma::sqrt(
                                arma::sum(
                                   arma::square(
                                      px),1));
            
            arma::vec sinRadsOnRads = arma::sin(rad)/rads;
            
            //project to the unit sphere
            result.col(0) = px.col(0)*sinRadsOnRads;
            result.col(1) = px.col(1)*sinRadsOnRads;
            result.col(2) = arma::cos(rads);
            
        } else if (rectilinear) {
            
            arma::vec2 imageCenter = arma::vec2({image.height/2,
                                                 image.width/2});
            
            double focalLength = image.lensParams[0];
            
            result.cols(0,1) = arma::conv_to<arma::mat>::from(pixels) - 
                                 arma::repmat(imageCenter.t(),1,pixels.n_rows) *
                                 pixelPitch;
            result.cols(2) = focalLength;
            
            result /= arma::repmat(arma::sqrt(arma::sum(arma::square(result),1)),3,1);
        }
    
        return std::move(result);
    }
    
    
}
}
}

#endif
