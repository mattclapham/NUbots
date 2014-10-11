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

/**
 *       @name LookUpTable
 *       @file lookuptable.h
 *       @brief Wraps LUT buffer with access methods for pixel classification
 *       @author Shannon Fenn
 *       @date 17-02-12
 *       @note ported by Jake Fountain Dec 2013 to NUClear system
 */

#ifndef MESSAGES_VISION_LOOKUPTABLE_H
#define MESSAGES_VISION_LOOKUPTABLE_H

#include <memory>
#include <string>
#include <cmath>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include "messages/input/Image.h"

namespace messages {
    namespace vision {

        enum Colour : char {
            // Main classifications
            UNCLASSIFIED = 'u',
            WHITE        = 'w',
            GREEN        = 'g',
            ORANGE       = 'o',
            YELLOW       = 'y',
            CYAN         = 'c',
            MAGENTA      = 'm',

            // Ambiguous Classifications
            WHITE_GREEN  = 'f'
        };

        class LookUpTable {
        public:
            uint8_t BITS_C1;
            uint8_t BITS_C2;
            uint8_t BITS_C3;
            size_t LUT_SIZE; //!< The size of a lookup table in bytes.

            LookUpTable();
            LookUpTable(uint8_t bitsC1, uint8_t bitsC2, uint8_t bitsC3, std::vector<Colour>&& data);

            std::string getData() const;
            const std::vector<Colour>& getRawData() const;

            /*!
                @brief Classifies a pixel
                @param p the pixel
                @return Returns the colour classification of this pixel
             */
            const messages::vision::Colour& operator()(const arma::Col<uint8_t>::fixed<3>& p) const;
            messages::vision::Colour& operator()(const arma::Col<uint8_t>::fixed<3>& p);

            /*!
             *   @brief Gets the index of the pixel in the LUT
             *   @param p The pixel to be classified.
             *   @return Returns the colour index for the given pixel.
             */
            uint getLUTIndex(const arma::Col<uint8_t>::fixed<3>& colour) const;

            /*!
             *   @brief The inverse of getLUTIndex
             *   NOTE: This inverse is NOT injective (e.g. not 1-to-1)
             */
            arma::Col<uint8_t>::fixed<3> getPixelFromIndex(const uint& index) const;
        private:

            uint8_t BITS_C1_REMOVED;
            uint8_t BITS_C2_REMOVED;
            uint8_t BITS_C3_REMOVED;
            uint8_t BITS_C2_C3;
            uint8_t BITS_C2_MASK;
            uint8_t BITS_C3_MASK;
            std::vector<Colour> data;
        };

        struct SaveLookUpTable {
        };

    } //vision
} // messages

// YAML conversions
namespace YAML {

    template<>
    struct convert<messages::vision::LookUpTable> {
        static Node encode(const messages::vision::LookUpTable& rhs) {
            Node node;

            node["bits"][0] = uint(rhs.BITS_C1);
            node["bits"][1] = uint(rhs.BITS_C2);
            node["bits"][2] = uint(rhs.BITS_C3);

            node["lut"] = rhs.getData();

            return node;
        }

        static bool decode(const Node& node, messages::vision::LookUpTable& rhs) {

            uint8_t bitsC1 = node["bits"][0].as<uint>();
            uint8_t bitsC2 = node["bits"][1].as<uint>();
            uint8_t bitsC3 = node["bits"][2].as<uint>();

            std::string dataString = node["lut"].as<std::string>();
            std::vector<messages::vision::Colour> data;

            data.reserve(dataString.size());
            for (auto& s : dataString) {
                data.push_back(messages::vision::Colour(s));
            }

            rhs = messages::vision::LookUpTable(bitsC1, bitsC2, bitsC3, std::move(data));

            return true;
        }
    };
}

#endif // MESSAGES_VISION_LOOKUPTABLE_H
