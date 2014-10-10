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

#define QUEX_SETTING_BUFFER_MIN_FALLBACK_N 0
#define QUEX_OPTION_ASSERTS_DISABLED
#define QUEX_OPTION_COMPUTED_GOTOS
#define QUEX_OPTION_TERMINATION_ZERO_DISABLED

namespace modules {
    namespace vision {

        template <typename Lexer>
        class QuexClassifier {
        private:
            static constexpr size_t BUFFER_SIZE = 2000;
            static __thread uint8_t buffer[BUFFER_SIZE]; // This should be big enough for now
            Lexer lexer;
            size_t& tknNumber;

        public:
            QuexClassifier();

            template <typename Iterator>
            std::vector<std::pair<uint32_t, uint32_t>> classify(Iterator begin, Iterator end) {

                // Start reading data
                lexer.buffer_fill_region_prepare();

                // Copy our data into the buffer
                std::copy(begin, end, buffer + 1);
                int length = std::distance(begin, end);

                lexer.buffer_fill_region_finish(length);

                std::vector<std::pair<uint32_t, uint32_t>> output;

                for(uint32_t typeID = lexer.receive();
                    typeID != QUEX_TKN_TERMINATION;
                    typeID = lexer.receive()) {

                    output.emplace_back({ typeID, len });
                }

                return output;
            }
        };
    }
}

#endif