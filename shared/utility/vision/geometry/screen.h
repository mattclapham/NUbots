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
#ifndef UTILITY_VISION_SCREEN_H
#define UTILITY_VISION_SCREEN_H

#include <cmath>
#include <armadillo>
#include <nuclear>
#include "messages/input/Image.h"

namespace utility {
namespace vision {
namespace geometry {

    using messages::input::Image;

    template <int camID>
    inline arma::mat camTiltMatrix(const Image<camID>& image) {
        //this returns an x-forward-aligned camera tilt matrix for filtering out of screen rays
        double cosxy = image.cameraToGround(0,0);
        double sinxy = image.cameraToGround(0,1);

        arma::mat camUnrotate;
        camUnrotate << cosxy << sinxy << 0.0 << arma::endr
                    << -sinxy << cosxy << 0.0 << arma::endr
                    << 0.0 << 0.0 << 1.0;
        return image.cameraToGround.submat(0,0,2,2)*camUnrotate;
    }

    template <int camID>
    inline arma::mat snapToScreen(const arma::mat& rayPositions, const arma::vec& rayLength, const Image<camID>& image) {
        //returns a mat of vectors of resized rayLength so that rays end at the edge of the image space

        arma::vec scales(rayPositions.n_rows);
        scales.fill(std::numeric_limits<double>::max());
        if (image.lens.type == Image<camID>::Lens::Type::RADIAL) {
            //radius and centre of circle
            const double pixelFOV = image.lens.parameters.radial.fov/2.0/image.lens.parameters.radial.pitch;
            arma::vec2 imageCenter = arma::vec2({image.lens.parameters.radial.centre[0],
                                                image.lens.parameters.radial.centre[1]});
            //ax^2 + bx + c parameters to solve
            const double a = arma::dot(rayLength,rayLength);
            const arma::vec b = 2.0 * ((rayPositions - arma::repmat(imageCenter.t(),rayPositions.n_rows,1)) * rayLength);
            const arma::vec c = arma::sum(rayPositions % rayPositions, 1) +
                                arma::accu(imageCenter % imageCenter) -
                                2.0 * (rayPositions * imageCenter) -
                                pixelFOV*pixelFOV;

            //the quadratic formula
            //std::cout << "A: " << a << std::endl;
            //std::cout << "B: " << b << std::endl;
            //std::cout << "C: " << c << std::endl;
            scales = ( -b + arma::sqrt(b % b - 4.0 * a * c) )/(2.0 * a);
        }

        //this makes sure the ray is on screen as well
        //check which direction to calculate the x edge of image intercept from
        if (rayLength[0] > 0) {
            //calculate the positive x intercept
            scales = arma::min((image.dimensions[0]/2-rayPositions.col(0))/rayLength[0], scales);

        } else if (rayLength[0] < 0) {
            //calculate the positive x intercept
            scales = arma::min( (rayPositions.col(0)-image.dimensions[0]/2)/rayLength[0], scales);
        }
        //now do y edge of image intercepts
        if (rayLength[0] == 0) {
            if (rayLength[1] > 0) {
                //calculate the positive x intercept
                scales = (image.dimensions[1]-rayPositions.col(1))/rayLength[1];

            } else if (rayLength[1] < 0) {
                //calculate the positive x intercept
                scales = (rayPositions.col(1)-image.dimensions[1])/rayLength[1];

            }
        } else {
            if (rayLength[1] > 0) {
                //calculate the positive x intercept
                scales = arma::min( (image.dimensions[1]-rayPositions.col(1))/rayLength[1], scales);

            } else if (rayLength[1] < 0) {
                //calculate the positive x intercept
                scales = arma::min( (rayPositions.col(1)-image.dimensions[1])/rayLength[1], scales);

            }
        }
        return arma::round( arma::repmat(rayLength,1,scales.n_rows) % arma::repmat(scales,1,2).t() + rayPositions.t() );
    }

    template <int camID>
    inline arma::imat trimToImage(const arma::imat& pixels, const Image<camID>& image) {
        //remove any pixel references not within the image area
        arma::imat result = pixels.rows(arma::find( (pixels.col(1) >= 0) % (pixels.col(1) < image.dimensions[1]) ));
        result = result.rows(arma::find( (result.col(0) >= 0) % (result.col(0) < image.dimensions[0]) ));
        return std::move(result);
    }

    template <int camID>
    inline arma::mat trimToFOV(const arma::mat& rays, const Image<camID>& image) {
        //remove any pixel references not within the screen FOV
        arma::uvec selected;
        if (image.lens.type == Image<camID>::Lens::Type::RADIAL) {
            //assuming Z is the forward vector
            const double fsize = cos(image.lens.parameters.radial.fov/2);
            selected = arma::find(rays.col(0) > fsize);

        } else if (image.lens.type == Image<camID>::Lens::Type::EQUIRECTANGULAR) {
            const double fsizex = cos(image.lens.parameters.equirectangular.fov[0]/2.0);
            const double fsizey = cos(image.lens.parameters.equirectangular.fov[1]/2.0);
            selected = arma::find( (rays.col(0) < fsizex) % (rays.col(1) < fsizey) );
        }
        return std::move(arma::mat(rays.rows(selected)));
    }

    template <int camID>
    inline arma::mat bulkRay2Pixel(const arma::mat& rays, const Image<camID>& image) {
        //convert camera rays to pixels
        arma::mat result;
        if (image.lens.type == Image<camID>::Lens::Type::RADIAL) {

            arma::vec2 imageCenter = arma::vec2({image.lens.parameters.radial.centre[0],
                                                image.lens.parameters.radial.centre[1]});
            result = rays.cols(1,2);

            arma::vec rads = arma::acos(rays.col(0));
            result /= arma::repmat(arma::sqrt(arma::sum(arma::square(result),1)),1,2);
            result %= arma::repmat(rads/image.lens.parameters.radial.pitch,1,2);
            result.col(1) *= -1.0;
            result += arma::repmat(imageCenter,1,rays.n_rows).t();

        } else if (image.lens.type == Image<camID>::Lens::Type::EQUIRECTANGULAR) {

            arma::vec2 imageCenter = arma::vec2({image.dimensions[1]/2.0,
                                                 image.dimensions[0]/2.0});

            result = rays.cols(0,1);

            result *= arma::repmat(image.lens.parameters.equirectangular.focalLength/rays.col(2),2,1);

            result += arma::repmat(imageCenter,1,rays.n_rows);
        }


        return arma::round(result);
    }

    template <int camID>
    inline arma::mat bulkPixel2Ray(const arma::imat& pixels, const Image<camID>& image) {
        //convert a matrix of rows of 2d pixels into spherical camera rays

        arma::mat result(pixels.n_rows,3);
        if (image.lens.type == Image<camID>::Lens::Type::RADIAL) {
            arma::vec imageCenter = arma::vec2({image.lens.parameters.radial.centre[0],
                                                image.lens.parameters.radial.centre[1]});
            //center the pixels
            const arma::mat px = (arma::conv_to<arma::mat>::from(pixels) -
                                 arma::repmat(imageCenter.t(),pixels.n_rows,1)) *
                                 image.lens.parameters.radial.pitch;
            //get all the radian values
            arma::vec rads = arma::sqrt(
                                arma::sum(
                                   arma::square(
                                      px),1));
            arma::vec sinRadsOnRads = arma::sin(rads)/rads;
            //project to the unit sphere
            result.col(1) = px.col(0) % sinRadsOnRads;
            result.col(2) = -px.col(1) % sinRadsOnRads;
            result.col(0) = cos(rads);
        } else if (image.lens.type == Image<camID>::Lens::Type::EQUIRECTANGULAR) {

            arma::vec2 imageCenter = arma::vec2({image.dimensions[1]/2.0,
                                                 image.dimensions[0]/2.0});

            result.cols(0,1) = arma::conv_to<arma::mat>::from(pixels) -
                                 arma::repmat(imageCenter.t(),1,pixels.n_rows);
            result.col(2) = image.lens.parameters.equirectangular.focalLength;

            result /= arma::repmat(arma::sqrt(arma::sum(arma::square(result),1)),3,1);
        }

        return std::move(result);
    }


}
}
}

#endif
