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

            enum class Format {
                YCbCr422,
                YCbCr444,
                RGB
            };

            struct Camera {
                std::function<arma::vec2 (const int& x, const int& y)> pixelAngle;
                std::function<arma::vec3 (const arma::vec2&)> project;
            };

            struct Pixel {
                uint8_t y;
                uint8_t cb;
                uint8_t cr;
            };

            inline Pixel& operator()(uint x, uint y) {
                return data[x + y * width()];
            }

            inline const Pixel& operator()(uint x, uint y) const {
                return data[x + y * width()];
            }

            inline uint width() const {
                return dimensions[0];
            }

            inline uint height() const {
                return dimensions[1];
            }

        public:
            NUClear::clock::time_point timestamp;
            Format format;
            arma::uvec2 dimensions;
            std::vector<Pixel> data;
            std::vector<uint8_t> source;
            Camera camera;
        };

    }  // input
}  // messages

#endif  // MESSAGES_INPUT_IMAGE_H
