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

#include "LinuxCamera.h"

extern "C" {
    #include <jpeglib.h>
}

#include "V4L2Camera.h"
#include "messages/input/Image.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace input {

        using messages::support::Configuration;

        // We assume that the device will always be video0, if not then change this
        LinuxCamera::LinuxCamera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // This trigger gets us as close as we can to the frame rate as possible (as high resolution as we can)
            on<Trigger<Every<V4L2Camera::FRAMERATE, Per<std::chrono::seconds>>>, Options<Single>>("Read Camera", [this](const time_t&) {

                // If the camera is ready, get an image and emit it
                if (camera.isStreaming()) {
                    auto image = camera.getImage();

                    // Apply our camera functions to this
                    image->camera.pixelAngle = pixelAngle;
                    image->camera.project = project;
                    emit(camera.getImage());
                }
            });

            // When we shutdown, we must tell our camera class to close (stop streaming)
            on<Trigger<Shutdown>>([this](const Shutdown&) {
                camera.closeCamera();
            });

            on<Trigger<Configuration<LinuxCamera>>>([this](const Configuration<LinuxCamera>& config) {

                double fovX = config["fov"][0].as<double>();
                double fovY = config["fov"][1].as<double>();

                pixelAngle = [](const int& x, const int& y) {
                    // Return the pixels angle from the camera
                    return arma::vec2({ 1, 2 });
                };

                project = [](const arma::vec2& angle) {
                    // Return the unit vector to the screen
                    return arma::vec3({ 1, 2, 3 });
                };

//                auto cameraParameters = std::make_unique<CameraParameters>();
//
//                cameraParameters->imageSizePixels << config["imageWidth"].as<uint>() << config["imageHeight"].as<uint>();
//                cameraParameters->FOV << config["FOV_X"].as<double>() << config["FOV_Y"].as<double>();
//                cameraParameters->distortionFactor = config["DISTORTION_FACTOR"].as<double>();
//                arma::vec2 tanHalfFOV;
//                tanHalfFOV << std::tan(cameraParameters->FOV[0] * 0.5) << std::tan(cameraParameters->FOV[1] * 0.5);
//                arma::vec2 imageCentre;
//                imageCentre << cameraParameters->imageSizePixels[0] * 0.5 << cameraParameters->imageSizePixels[1] * 0.5;
//                cameraParameters->pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]) << (tanHalfFOV[1] / imageCentre[1]);
//                cameraParameters->focalLengthPixels = imageCentre[0] / tanHalfFOV[0];
//
//
//                emit<Scope::DIRECT>(std::move(cameraParameters));

                try {
                    // Recreate the camera device at the required resolution
                    int width = config["image_width"].as<uint>();
                    int height = config["image_height"].as<uint>();
                    std::string devicePath = config["device_path"].as<std::string>();
                    std::string format = config["imageFormat"].as<std::string>();

                    if (camera.getWidth() != static_cast<size_t>(width)
                        || camera.getHeight() != static_cast<size_t>(height)
                        || camera.getFormat() != format
                        || camera.getDevicePath() != devicePath) {
                        camera.resetCamera(devicePath, format, width, height);
                    }

                    // Set all other camera settings
                    for(auto& setting : camera.getSettings()) {
                        int value = config[setting.first].as<int>();
                        if(setting.second.set(value) == false) {
                            NUClear::log<NUClear::DEBUG>("Failed to set " + setting.first + " on camera");
                        }
                    }

                    // Start the camera streaming video
                    camera.startStreaming();
                } catch(const std::exception& e) {
                    NUClear::log<NUClear::DEBUG>(std::string("Exception while setting camera configuration: ") + e.what());
                    throw e;
                }
            });

            // Try to reapply the camera settings every 1 second because sometimes they don't all apply properly
            on<Trigger<Every<1, std::chrono::seconds>>, With<Configuration<LinuxCamera>>>("Camera Setting Applicator", [this] (const time_t&, const Configuration<LinuxCamera>& config) {
                if(camera.isStreaming()) {
                    // Set all other camera settings
                    for(auto& setting : camera.getSettings()) {
                        int value = config[setting.first].as<int>();
                        if(setting.second.set(value) == false) {
                            NUClear::log<NUClear::DEBUG>("Failed to set " + setting.first + " on camera");
                        }
                    }
                }
            });
        }

    }  // input
}  // modules
