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

#include "LookUpTable.h"
#include <fstream>
#include <cstring>
#include <iostream>

namespace messages {
    namespace vision {

        LookUpTable::LookUpTable(uint8_t bitsY, uint8_t bitsCb, uint8_t bitsCr, std::vector<Colour>&& data)
            : BITS_C1(bitsY)
            , BITS_C2(bitsCb)
            , BITS_C3(bitsCr)
            , LUT_SIZE(1 << (BITS_C1 + BITS_C2 + BITS_C3))
            , BITS_C1_REMOVED(sizeof(uint8_t) * 8 - BITS_C1)
            , BITS_C2_REMOVED(sizeof(uint8_t) * 8 - BITS_C2)
            , BITS_C3_REMOVED(sizeof(uint8_t) * 8 - BITS_C3)
            , BITS_C2_C3(BITS_C2 + BITS_C3)
            , BITS_C2_MASK(std::pow(2, BITS_C2) - 1)
            , BITS_C3_MASK(std::pow(2, BITS_C3) - 1)
            , data(std::move(data)) {
        }

        LookUpTable::LookUpTable() {
        }

        const Colour& LookUpTable::operator()(const arma::Col<uint8_t>::fixed<3>& p) const {
            return data[getLUTIndex(p)];
        }

        Colour& LookUpTable::operator()(const arma::Col<uint8_t>::fixed<3>& p) {
            return data[getLUTIndex(p)];
        }

        std::string LookUpTable::getData() const {
            return std::string(data.begin(), data.end());
        }

        const std::vector<Colour>& LookUpTable::getRawData() const {
            return data;
        }

        uint LookUpTable::getLUTIndex(const arma::Col<uint8_t>::fixed<3>& colour) const {
            unsigned int index = 0;

            index += ((colour[0] >> BITS_C1_REMOVED) << BITS_C2_C3);
            index += ((colour[1] >> BITS_C2_REMOVED) << BITS_C3);
            index +=  (colour[2] >> BITS_C3_REMOVED);

            return index;
        }

        arma::Col<uint8_t>::fixed<3> LookUpTable::getPixelFromIndex(const uint& index) const {
            uint8_t c1 = (index >> BITS_C2_C3) << BITS_C1_REMOVED;
            uint8_t c2 = ((index >> BITS_C3) & BITS_C2_MASK) << BITS_C2_REMOVED;
            uint8_t c3 = (index & BITS_C3_MASK) << BITS_C3_REMOVED;

            return {c1, c2, c3};
        }

    } //vision
} // messages
