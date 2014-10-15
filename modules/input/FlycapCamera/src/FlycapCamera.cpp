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

#include "FlycapCamera.h"

#include "utility/image/ColorModelConversions.h"
#include "messages/support/Configuration.h"
#include "CamCallbacks.h"

namespace modules
{
namespace input
{

using messages::support::Configuration;

FlycapCamera::FlycapCamera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment))
{

    // When we shutdown, we must tell our camera class to close (stop streaming)
    on<Trigger<Shutdown>>([this](const Shutdown &)
    {

        // Stop streaming somehow
        // for (auto& camera: cameras) {
        //     camera.closeCamera();
        // }
    });

    on<Trigger<Configuration<FlycapCamera>>>([this](const Configuration<FlycapCamera> &config)
    {
        try
        {
            // Try to find our camera
            uint deviceId = config["device_id"].as<int>();
            auto camera = cameras.find(deviceId);

            // If we don't have a camera then make a new one
            if (camera == cameras.end()) {

                // Stop all the cameras streaming
                for (auto &cam : cameras) {
                    cam.second.second->StopCapture();
                }

                // Make a new camera
                auto newCam = std::make_unique<FlyCapture2::Camera>();

                // Find the physical camera to connect to
                FlyCapture2::PGRGuid id;
                FlyCapture2::BusManager().GetCameraFromSerialNumber(deviceId, &id);
                FlyCapture2::Error error = newCam->Connect(&id);

                if (error != FlyCapture2::PGRERROR_OK) {
                    throw std::system_error(errno, std::system_category(), "Failed to connect to camera, did you run as sudo?");
                }

                // Set our camera settings
                error = newCam->SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_3_75);
                if (error != FlyCapture2::PGRERROR_OK) {
                    throw std::system_error(errno, std::system_category(), "Failed to set the format or framerate");
                }

                // Insert our new camera
                camera = cameras.insert(std::make_pair(deviceId, std::make_pair(Image::Lens(), std::move(newCam)))).first;

                // Stop all the cameras streaming
                for (auto &cam : cameras) {
                    cam.second.second->StartCapture();
                }
            }

            auto& cam = *camera->second.second;
            auto& lens = camera->second.first;

            if(config["lens"]["type"].as<std::string>() == "RADIAL") {
                lens.type = Image::Lens::Type::RADIAL;
                lens.parameters.radial.fov = config["lens"]["fov"].as<double>();
                lens.parameters.radial.pitch = config["lens"]["pixel_pitch"].as<double>();
                lens.parameters.radial.centre[0] = config["lens"]["image_centre"][0].as<double>();
                lens.parameters.radial.centre[1] = config["lens"]["image_centre"][0].as<double>();
            }

            FlyCapture2::Property p;
            p.type = FlyCapture2::BRIGHTNESS;
            cam.GetProperty(&p);
            p.onOff = true;
            p.valueA = config["brightness"].as<unsigned int>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::AUTO_EXPOSURE;
            cam.GetProperty(&p);
            p.onOff = config["auto_exposure"].as<bool>();
            p.absValue = config["auto_exposure_val"].as<float>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::WHITE_BALANCE;
            cam.GetProperty(&p);
            p.valueA = config["white_balance_temperature_red"].as<unsigned int>();
            p.valueB = config["white_balance_temperature_blue"].as<unsigned int>();
            p.onOff = config["auto_white_balance"].as<bool>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::GAMMA;
            cam.GetProperty(&p);
            p.onOff = true;
            p.valueA = config["gamma"].as<unsigned int>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::PAN;
            cam.GetProperty(&p);
            p.valueA = config["absolute_pan"].as<unsigned int>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::TILT;
            cam.GetProperty(&p);
            p.valueA = config["absolute_tilt"].as<unsigned int>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::SHUTTER;
            cam.GetProperty(&p);
            p.valueA = config["absolute_exposure"].as<unsigned int>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::GAIN;
            cam.GetProperty(&p);
            p.autoManualMode = config["gain_auto"].as<bool>();
            p.valueA = config["gain"].as<unsigned int>();
            cam.SetProperty(&p);

            p.type = FlyCapture2::TEMPERATURE;
            cam.GetProperty(&p);
            p.valueA = config["white_balance_temperature_red"].as<unsigned int>();
            p.valueB = config["white_balance_temperature_blue"].as<unsigned int>();
            p.onOff = config["auto_white_balance"].as<bool>();
            cam.SetProperty(&p);

            FlyCapture2::FC2Config camConf;
            cam.GetConfiguration(&camConf);
            camConf.numBuffers = 3;
            camConf.highPerformanceRetrieveBuffer = true;
            camConf.grabTimeout = 500;
            cam.SetConfiguration(&camConf);
        }
        catch (const std::exception &e) {
            NUClear::log<NUClear::DEBUG>(std::string("Exception while starting camera streaming: ") + e.what());
            throw e;
        }
    });

    on<Trigger<Every<225, Per<std::chrono::minutes>>>, Options<Single>>([this](const time_t&) {

        FlyCapture2::Image image;

        for(const auto& camera : cameras) {
            auto& cam = *camera.second.second;
            auto& lens = camera.second.first;

            cam.RetrieveBuffer(&image);

            auto img = std::make_unique<Image>(captureBayer(image));
            img->lens = lens;

            emit(std::move(img));
        }
    });
}

}  // input
}  // modules
