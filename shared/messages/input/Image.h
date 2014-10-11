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

                    } equirectangular;

                    struct radial {

                    } radial;
                } parameters;
            };

            inline arma::Col<uint8_t>::fixed<3> operator()(uint x, uint y) const {
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
        };

    }  // input
}  // messages

#endif  // MESSAGES_INPUT_IMAGE_H
