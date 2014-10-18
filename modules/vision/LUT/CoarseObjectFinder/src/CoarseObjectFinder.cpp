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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "CoarseObjectFinder.h"

#include "messages/vision/VisualHorizon.h"

namespace modules {
namespace vision {
namespace LUT {

    using messages::vision::VisualHorizon;
    using messages::vision::

    CoarseObjectFinder::CoarseObjectFinder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Trigger<Raw<VisualHorizon>>>([this](const std::shared_ptr<const VisualHorizon>& input) {


            auto points =


            auto scan = std::make_unique<ImagePointScan>();

            scan->horizon = input;
            scan->points = scanner.findObjects(*input->image, *input->lut, input->horizon);

            VisualHorizon horizon;
            std::map<uint32_t, arma::vec3> points;

            // Call the functions here

            // input->image is image
            // input->horizon is visual horizon
        });

    }

}
}
}

