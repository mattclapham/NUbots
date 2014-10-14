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

#ifndef UTILITY_VISION_QUEXCLASSIFIER_H
#define UTILITY_VISION_QUEXCLASSIFIER_H

#include <vector>
#include <armadillo>

#include "messages/input/Image.h"

namespace utility {
    namespace vision {


        inline std::vector<arma::ivec2> bresenhamLine(const arma::ivec2& start, const arma::ivec2& end) {

            std::vector<arma::ivec2> out;

            arma::ivec2 point = start;

            // The distance to travel with 1 point of fixed precision
            arma::ivec2 delta = (end - point) * 2;

            // Calculate direction of movement
            arma::ivec2 d = arma::sign(delta);

            out.push_back(point);

            // Which axis we move on
            bool movement = delta[0] >= delta[1];

            int error = delta[!movement] - (delta[movement] >> 1);
            while(point[movement] != end[movement]) {

                if ((error >= 0) && (error || (d[movement] > 0))) {
                    error -= delta[movement];
                    point[!movement] += d[!movement];
                }

                error += delta[!movement];
                point[movement] += d[movement];

                out.push_back(point);

            }

            return out;
        }

        template <typename Lexer, typename Iterator>
        std::vector<std::pair<uint32_t, uint32_t>> quexClassify(Iterator start, Iterator end) {

            constexpr int BUFFER_SIZE = 2000;

            uint8_t buffer[BUFFER_SIZE];

            // Thread safety make a new lexer every time :( (i don't know how expensive this is)
            Lexer lexer(buffer, BUFFER_SIZE, buffer + 1);

            // Copy our data into the buffer
            std::copy(start, end, buffer + 1);
            int length = std::distance(start, end);

            lexer.buffer_fill_region_finish(length);

            std::vector<std::pair<uint32_t, uint32_t>> output;

            size_t& len(lexer.token_p()->number);

            for(uint32_t typeID = lexer.receive();
                typeID != QUEX_TKN_TERMINATION;
                typeID = lexer.receive()) {

                output.emplace_back(std::make_pair(uint(typeID), uint(len)));
            }

            return output;

        }
    }
}

#endif