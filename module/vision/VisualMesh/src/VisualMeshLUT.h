/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_VISION_VISUALMESHLUT_H
#define MODULE_VISION_VISUALMESHLUT_H

#include "message/vision/MeshObjectRequest.h"
#include <utility>
#include <vector>
#include <cstddef>

namespace module {
namespace vision {

    class VisualMeshLUT {
    public:
        VisualMeshLUT(double minHeight, double maxHeight, int slices = 50);

        void addShape(const message::vision::MeshObjectRequest& request);

        void regenerate();
        std::pair<std::vector<std::pair<double, double>>::iterator, std::vector<std::pair<double, double>>::iterator> getLUT(double height, double minPhi, double maxPhi);

    private:
        size_t slices;
        double minHeight;
        double maxHeight;
        std::vector<message::vision::MeshObjectRequest> requests;
        std::vector<std::vector<std::pair<double, double>>> luts;
    };

}
}

#endif  // MODULE_VISION_VISUALMESHLUT_H
