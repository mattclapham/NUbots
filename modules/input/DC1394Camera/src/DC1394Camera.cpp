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
#include "utility/error/dc1394_error_category.h"

namespace modules {
namespace input {

    using messages::support::Configuration;

    DC1394Camera::DC1394Camera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)),
        context(dc1394_new(), [](dc1394_t* ptr) {
            dc1394_free(ptr);
        }) {

        const int NUM_BUFFERS = 2;

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

                // Our error variable
                dc1394error_t err;

                // Set some important bus options
                err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_400);
                if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                err = dc1394_video_set_mode(newCam.get(), DC1394_VIDEO_MODE_1280x960_RGB8);
                if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                err = dc1394_video_set_framerate(newCam.get(), DC1394_FRAMERATE_7_5);
                if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                // Setup our capture
                err = dc1394_capture_setup(newCam.get(), NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
                if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                // Start the camera
                err = dc1394_video_set_transmission(newCam.get(), DC1394_ON);
                if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                // Add our camera to the list
                cameras.insert(std::make_pair(deviceId, std::move(newCam)));

            }

            // TODO Apply our settings
        });
    }

}
}

