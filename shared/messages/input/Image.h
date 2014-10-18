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

#ifndef MESSAGES_INPUT_IMAGE_H
#define MESSAGES_INPUT_IMAGE_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <nuclear>
#include <armadillo>

namespace messages {
    namespace input {

        class Image {
        public:

            enum class SourceFormat {
                YCbCr422,
                YCbCr444,
                RGB,
                JPEG,
                BGGR
            };

            struct Lens {
                enum class Type {
                    EQUIRECTANGULAR,
                    RADIAL,
                    BARREL
                } type;

                union {
                    struct equirectangular {
                        double fov[2];
                        double focalLength;
                    } equirectangular;

                    struct radial {
                        double fov;
                        double pitch;
                        double centre[2];
                    } radial;
                } parameters;
            };



            function get(x, y) {
            if (x < 0 || x >= width || y < 0 || y >= height) {
                return null;
            }
            return data[y * width + x];
        }
        function getAvg() {
            var sum = 0;
            var count = 0;
            for (var i = 0; i < arguments.length; i++) {
                var coord = arguments[i];
                var color = get(coord[0], coord[1]);
                if (color !== null) {
                    sum += color;
                    count++;
                }
            }
            return sum / count;
        }

            inline char get(uint x, uint y) {

            }

            inline arma::Col<uint8_t>::fixed<3> operator()(const uint& x, const uint& y) const {

                arma::Col<uint8_t>::fixed<3> output;

                bool oX = x % 2;
                bool oY = y % 2;

                if(oX != oY) {
                    output[ oY * 3] = get(x - 1, y) + get(x + 1, y); // Left right
                    output[2]       = get(x - 1, y - 1), get(); // all 5
                    output[!oY * 3] = get(x, y - 1) + get(x + 1, y + 1); // Top base
                }
                else {
                    output[ oY * 3] = get(x, y); // Centre
                    output[2]       = ; // Top/Bottom/Left/Right
                    output[!oY * 3] = ; // diags
                    // 4 search
                }

                if (y % 2 === 0) {
                    // B G
                    if (x % 2 === 1) {
                        // B

                        data[y * width + x]



                        output[0] = getAvg([x - 1, y - 1], [x + 1, y - 1], [x + 1, y + 1], [x - 1, y + 1]); // 4 surrounding
                        output[1] = getAvg([x, y - 1], [x + 1, y], [x, y + 1], [x - 1, y]); // 4 surrounding
                        output[2] = get(x, y); // self
                    } else {
                        // G
                        output[0] = getAvg([x, y - 1], [x, y + 1]); // 2 surrounding
                        output[1] = getAvg([x - 1, y - 1], [x + 1, y - 1], [x + 1, y + 1], [x - 1, y + 1], [x, y]); // 5 surrounding
                        output[2] = getAvg([x - 1, y], [x + 1, y]); // 2 surrounding
                    }
                } else {
                    // G R
                    if (x % 2 === 1) {
                        // G
                        output[0] = getAvg([x - 1, y], [x + 1, y]); // 2 surrounding
                        output[1] = getAvg([x - 1, y - 1], [x + 1, y - 1], [x + 1, y + 1], [x - 1, y + 1], [x, y]); // 5 surrounding
                        output[2] = getAvg([x, y - 1], [x, y + 1]); // 2 surrounding
                    } else {
                        // R
                        output[0] = get(x, y); // self
                        output[1] = getAvg([x, y - 1], [x + 1, y], [x, y + 1], [x - 1, y]); // 4 surrounding
                        output[2] = getAvg([x - 1, y - 1], [x + 1, y - 1], [x + 1, y + 1], [x - 1, y + 1]); // 4 surrounding
                    }
                }

                //DEBAYERSZEFOISEJFOISEJFOISEJFOPISEJ


                return {0, 0, 0};//source[x + y * width()]; // TODO do the bayer conversion here
            }

            inline uint width() const {
                return dimensions[0];
            }

            inline uint height() const {
                return dimensions[1];
            }

        public:
            NUClear::clock::time_point timestamp;
            SourceFormat format;
            arma::uvec2 dimensions;
            std::vector<uint8_t> source;
            Lens lens;
            arma::mat44 cameraToGround;
        };

    }  // input
}  // messages

#endif  // MESSAGES_INPUT_IMAGE_H
