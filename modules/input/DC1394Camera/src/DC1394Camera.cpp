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

#include "DC1394Camera.h"

#include "messages/support/Configuration.h"

namespace modules {
namespace input {

    using messages::support::Configuration;

    DC1394Camera::DC1394Camera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)),
        context(dc1394_new(), [](dc1394_t* ptr) {
            dc1394_free(ptr);
        }) {

        on<Trigger<Configuration<DC1394Camera>>>([this](const Configuration<DC1394Camera> &config) {

            // Get the device
            uint64_t deviceId = config["device"].as<uint64_t>();

            // See if we already have this camera
            auto camera = cameras.find(deviceId);

            // If we don't have a camera then make a new one
            if (camera == cameras.end()) {

                // Stop all the cameras streaming

                // Make a new camera
                auto newCam = std::unique_ptr<dc1394camera_t, std::function<void (dc1394camera_t*)>>(dc1394_camera_new(context.get(), deviceId), [](dc1394camera_t* ptr) {
                    dc1394_camera_free(ptr);
                });

                // DO SOME SETUP THINGS

                cameras.insert(std::make_pair(deviceId, std::move(newCam)));

            }
        });
    }

}
}

