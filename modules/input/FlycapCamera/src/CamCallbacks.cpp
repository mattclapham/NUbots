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

#include "CamCallbacks.h"

#include <cmath>
#include "FlycapCamera.h"
#include "messages/input/Image.h"

namespace modules {
    namespace input {

        using messages::input::Image;

        //these are used for radial lenses with a circular display area to speed up the image demosaicing
        //get the factor of 2 aligned left edge of the circle
        static constexpr size_t getViewStart(const int& ptHeight, const int& width, const int& height, const int& radius) {
            return static_cast<size_t>(((width/2 - (int)sqrt(radius*radius - (ptHeight-height/2)*(ptHeight-height/2))) / 2) * 2);
        }

        //get the factor of 2 aligned right edge of the circle
        static constexpr size_t getViewEnd(const int& ptHeight, const int& width, const int& height, const int& radius) {
            return static_cast<size_t>(((width/2 + (int)sqrt(radius*radius - (ptHeight-height/2)*(ptHeight-height/2))) / 2) * 2);
        }

        Image<0> captureBayer(FlyCapture2::Image& image) {

            // Return an image with bayer

            uint size = image.GetDataSize();

            std::vector<uint8_t> data(size);

            std::memcpy(data.data(), reinterpret_cast<char*>(image.GetData()), size);

            Image<0> output;
            output.timestamp = NUClear::clock::now();
            output.format =  Image<0>::SourceFormat::BGGR;
            output.dimensions = { 1280, 960 };
            output.source = std::move(data);

            return output;
        }

        /**
         * @brief This class encapsulates the physical camera device. It will setup a camera device and begin streaming
         *    images
         *
         * @details
         *    This callback function converts a radial lens Bayer format image to rgb to work with NUClear.
         *
         * @author Josiah Walker
         */
        // Image captureRadial(FlyCapture2::Image& image) {

        //     constexpr uint sourceWidth = 1280;
        //     constexpr uint sourceHeight = 960;

        //     //the horizontal offset cuts out the black areas of the image altogether to save CPU
        //     std::vector<Image::Pixel> data(sourceHeight*(sourceWidth), {0,0,0});

        //     // do a cache-coherent demosaic step
        //     for (size_t j = 0; j < sourceHeight*sourceWidth; j += sourceWidth) {

        //         // do the second line
        //         for (size_t i = 0;
        //             i < 1280-2; i += 2) { // assume we always start on an even pixel (odd ones are nasty)

        //             const size_t index = i+j;
        //             const size_t dIndex = index+1;
        //             //do the current row
        //             auto& pxNext = data[dIndex];
        //             auto& pxNextNext = data[dIndex+1];

        //             //get the required information
        //             const auto& currentGreen = *image[index];
        //             const auto& currentRed = *image[index+1];
        //             const auto& nextGreen = *image[index+2];
        //             const auto& nextRed = *image[index+3];

        //             //demosaic red and green

        //             pxNext.cb = uint8_t((uint(currentGreen) + uint(nextGreen)) >> 1);
        //             pxNext.cr = currentRed;
        //             pxNextNext.cb = nextGreen;
        //             pxNextNext.cr = uint8_t((uint(currentRed) + uint(nextRed)) >> 1);

        //             //do the row below
        //             data[dIndex+sourceWidth].cr = currentRed;
        //             data[dIndex+1+sourceWidth].cr = uint8_t((uint(currentRed) + uint(nextRed)) >> 1);

        //         }
        //         j += sourceWidth;

        //         for (size_t i = 0;
        //             i < 1280-2; i += 2) { // assume we always start on an even pixel (odd ones are nasty)

        //             const size_t index = i+j;
        //             const size_t dIndex = index+1;
        //             //do the current row
        //             auto& pxNext = data[dIndex];
        //             auto& pxNextNext = data[dIndex+1];

        //             //get the required information
        //             const auto& currentBlue = *image[index];
        //             const auto& currentGreen = *image[index+1];
        //             const auto& nextBlue = *image[index+2];
        //             const auto& nextGreen = *image[index+3];

        //             //demosaic red and green
        //             pxNext.y = uint8_t((uint(currentBlue) + uint(nextBlue)) >> 1);
        //             pxNext.cb = currentGreen;
        //             pxNextNext.y = nextBlue;
        //             pxNextNext.cb = uint8_t((uint(currentGreen) + uint(nextGreen)) >> 1);

        //             //do the row below
        //             //px = data[dIndex+radius*2];
        //             data[dIndex+sourceWidth].y = (unsigned char)(((unsigned int)currentBlue + (unsigned int)nextBlue) >> 1);
        //             data[dIndex+1+sourceWidth].y = nextBlue;

        //         }

        //     }
        //     return Image(sourceWidth, sourceHeight, std::move(data));
        // }

    }  // input
}  // modules


