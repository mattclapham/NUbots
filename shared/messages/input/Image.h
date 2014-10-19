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

            inline char get(const uint& x, const uint& y) const {
                return source[y * width() + x];
            }

            inline arma::Col<uint8_t>::fixed<3> operator()(const uint& x, const uint& y) const {

                arma::Col<uint8_t>::fixed<3> output;

                bool oX = x % 2;
                bool oY = y % 2;

                if(oX != oY) {
                    output[ oY * 3] = (get(x - 1, y) + get(x + 1, y))                                                             / 2; // Left right
                    output[2]       = (get(x, y) + get(x - 1, y - 1) + get(x + 1, y - 1) + get(x + 1, y + 1) + get(x - 1, y + 1)) / 5; // Diag + mid
                    output[!oY * 3] = (get(x, y - 1) + get(x, y + 1))                                                             / 2; // Top base
                }
                else {
                    output[ oY * 3] = get(x, y); // Centre
                    output[2]       = (get(x, y - 1) + get(x, y + 1) + get(x - 1, y) + get(x + 1, y))                             / 4; // Top/Bottom/Left/Right
                    output[!oY * 3] = (get(x - 1, y - 1) + get(x + 1, y - 1) + get(x + 1, y + 1) + get(x - 1, y + 1))             / 4; // diags
                }

                return output;
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
