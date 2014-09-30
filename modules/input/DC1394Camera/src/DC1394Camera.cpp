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

#include "messages/input/Image.h"
#include "messages/support/Configuration.h"
#include "utility/error/dc1394_error_category.h"

namespace modules {
namespace input {

    using messages::support::Configuration;
    using messages::input::Image;

    DC1394Camera::DC1394Camera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)),
        context(dc1394_new(), dc1394_free) {

        const int NUM_BUFFERS = 4;

        on<Trigger<Configuration<DC1394Camera>>>([this](const Configuration<DC1394Camera>& config) {

            // Our error variable
            dc1394error_t err;

            // Get the device
            uint64_t deviceId = config["device"].as<uint64_t>();

            // See if we already have this camera
            auto camera = cameras.find(deviceId);

            // If we don't have a camera then make a new one
            if (camera == cameras.end()) {

                // Stop all the cameras streaming

                std::cout << config.name << std::endl;

                // Make a new camera
                auto newCam = std::unique_ptr<dc1394camera_t, std::function<void (dc1394camera_t*)>>(dc1394_camera_new(context.get(), deviceId), dc1394_camera_free);

                if(newCam) {

                    // Set some important bus options
                    err = dc1394_video_set_operation_mode(newCam.get(), DC1394_OPERATION_MODE_LEGACY);
                    if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                    err = dc1394_video_set_iso_speed(newCam.get(), DC1394_ISO_SPEED_400);
                    if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                    err = dc1394_video_set_mode(newCam.get(), DC1394_VIDEO_MODE_1280x960_RGB8);
                    if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                    err = dc1394_video_set_framerate(newCam.get(), DC1394_FRAMERATE_3_75);
                    if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                    // Setup our capture
                    err = dc1394_capture_setup(newCam.get(), NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
                    if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());


                    // Get the power settings
                    // dc1394_software_trigger_set_power(newCam.get(), DC1394_OFF);
                    // dc1394switch_t value;
                    // dc1394_software_trigger_get_power(newCam.get(), &value);
                    // std::cout << value << std::endl;
                    // dc1394_external_trigger_get_power(newCam.get(), &value);
                    // std::cout << value << std::endl;

                    // Start the camera
                    err = dc1394_video_set_transmission(newCam.get(), DC1394_ON);
                    if(err > 0) throw std::system_error(err, utility::error::dc1394_error_category());

                    dc1394switch_t value;
                    err = dc1394_video_get_transmission(newCam.get(), &value);
                    std::cout << "Transmitting " << value << std::endl;

                    // Reset to factory default settings
                    dc1394_memory_load(newCam.get(), 0);

                    // Print info about the camera
                    dc1394_camera_print_info(newCam.get(), stdout);

                    // Add our camera to the list
                    cameras.insert(std::make_pair(deviceId, std::move(newCam)));
                }
                else {
                    // TODO output an error
                    std::cout << "An error occured" << std::endl;
                }



                // Apparently ISO speed is transmission speed in MB/s and must be set
                /*
                  DC1394_ISO_SPEED_100= 0,
                  DC1394_ISO_SPEED_200,
                  DC1394_ISO_SPEED_400,
                  DC1394_ISO_SPEED_800,
                  DC1394_ISO_SPEED_1600,
                  DC1394_ISO_SPEED_3200
                 */

                // The settings (features) note that exposure is autoexposure
                /*
                  DC1394_FEATURE_BRIGHTNESS= 416,
                  DC1394_FEATURE_EXPOSURE,
                  DC1394_FEATURE_SHARPNESS,
                  DC1394_FEATURE_WHITE_BALANCE,
                  DC1394_FEATURE_HUE,
                  DC1394_FEATURE_SATURATION,
                  DC1394_FEATURE_GAMMA,
                  DC1394_FEATURE_SHUTTER,
                  DC1394_FEATURE_GAIN,
                  DC1394_FEATURE_IRIS,
                  DC1394_FEATURE_FOCUS,
                  DC1394_FEATURE_TEMPERATURE,
                  DC1394_FEATURE_TRIGGER,
                  DC1394_FEATURE_TRIGGER_DELAY,
                  DC1394_FEATURE_WHITE_SHADING,
                  DC1394_FEATURE_FRAME_RATE,
                  DC1394_FEATURE_ZOOM,
                  DC1394_FEATURE_PAN,
                  DC1394_FEATURE_TILT,
                  DC1394_FEATURE_OPTICAL_FILTER,
                  DC1394_FEATURE_CAPTURE_SIZE,
                  DC1394_FEATURE_CAPTURE_QUALITY
                 */

                // Framerates
                /*
                  DC1394_FRAMERATE_1_875= 32,
                  DC1394_FRAMERATE_3_75,
                  DC1394_FRAMERATE_7_5,
                  DC1394_FRAMERATE_15,
                  DC1394_FRAMERATE_30,
                  DC1394_FRAMERATE_60,
                  DC1394_FRAMERATE_120,
                  DC1394_FRAMERATE_240
                 */


                // Error codes
                /*
                  DC1394_SUCCESS=0,   // Success is zero
                  DC1394_FAILURE,     // Errors are positive numbers
                  DC1394_NO_FRAME=-2, // Warnings or info are negative numbers
                  DC1394_NO_CAMERA=3,
                  DC1394_NOT_A_CAMERA,
                  DC1394_FUNCTION_NOT_SUPPORTED,
                  DC1394_CAMERA_NOT_INITITIALIZED,
                  DC1394_INVALID_FEATURE,
                  DC1394_INVALID_FORMAT,
                  DC1394_INVALID_MODE,
                  DC1394_INVALID_FRAMERATE,
                  DC1394_INVALID_TRIGGER_MODE,
                  DC1394_INVALID_TRIGGER_SOURCE,
                  DC1394_INVALID_ISO_SPEED,
                  DC1394_INVALID_IIDC_VERSION,
                  DC1394_INVALID_COLOR_MODE,
                  DC1394_INVALID_FORMAT7_COLOR_TILE,
                  DC1394_REQ_VALUE_OUTSIDE_RANGE,
                  DC1394_INVALID_ERROR_CODE,
                  DC1394_MEMORY_ALLOCATION_FAILURE,
                  DC1394_TAGGED_REGISTER_NOT_FOUND,
                  DC1394_FORMAT7_ERROR_FLAG_1,
                  DC1394_FORMAT7_ERROR_FLAG_2,
                  DC1394_INVALID_BAYER_METHOD,
                  DC1394_HANDLE_CREATION_FAILURE,
                  DC1394_GENERIC_INVALID_ARGUMENT,
                  DC1394_INVALID_VIDEO1394_DEVICE,
                  DC1394_NO_ISO_CHANNEL,
                  DC1394_NO_BANDWIDTH,
                  DC1394_IOCTL_FAILURE,
                  DC1394_CAPTURE_IS_NOT_SET = -30,
                  DC1394_CAPTURE_IS_RUNNING = 31,
                  DC1394_RAW1394_FAILURE,
                  DC1394_BASLER_NO_MORE_SFF_CHUNKS,
                  DC1394_BASLER_CORRUPTED_SFF_CHUNK,
                  DC1394_BASLER_UNKNOWN_SFF_CHUNK
                 */

                // There are feature modes for features
                /*
                  DC1394_FEATURE_MODE_MANUAL= 0,
                  DC1394_FEATURE_MODE_AUTO,
                  DC1394_FEATURE_MODE_ONE_PUSH_AUTO
                */

                // There are several colour modes
                /*
                  DC1394_COLOR_CODING_MONO8= 320,
                  DC1394_COLOR_CODING_YUV411,
                  DC1394_COLOR_CODING_YUV422,
                  DC1394_COLOR_CODING_YUV444,
                  DC1394_COLOR_CODING_RGB8,
                  DC1394_COLOR_CODING_MONO16,
                  DC1394_COLOR_CODING_RGB16,
                  DC1394_COLOR_CODING_MONO16S,
                  DC1394_COLOR_CODING_RGB16S,
                  DC1394_COLOR_CODING_RAW8,
                  DC1394_COLOR_CODING_RAW16
                 */

                // Video modes
                /*
                  DC1394_VIDEO_MODE_160x120_YUV444= 64,
                  DC1394_VIDEO_MODE_320x240_YUV422,
                  DC1394_VIDEO_MODE_640x480_YUV411,
                  DC1394_VIDEO_MODE_640x480_YUV422,
                  DC1394_VIDEO_MODE_640x480_RGB8,
                  DC1394_VIDEO_MODE_640x480_MONO8,
                  DC1394_VIDEO_MODE_640x480_MONO16,
                  DC1394_VIDEO_MODE_800x600_YUV422,
                  DC1394_VIDEO_MODE_800x600_RGB8,
                  DC1394_VIDEO_MODE_800x600_MONO8,
                  DC1394_VIDEO_MODE_1024x768_YUV422,
                  DC1394_VIDEO_MODE_1024x768_RGB8,
                  DC1394_VIDEO_MODE_1024x768_MONO8,
                  DC1394_VIDEO_MODE_800x600_MONO16,
                  DC1394_VIDEO_MODE_1024x768_MONO16,
                  DC1394_VIDEO_MODE_1280x960_YUV422,
                  DC1394_VIDEO_MODE_1280x960_RGB8,
                  DC1394_VIDEO_MODE_1280x960_MONO8,
                  DC1394_VIDEO_MODE_1600x1200_YUV422,
                  DC1394_VIDEO_MODE_1600x1200_RGB8,
                  DC1394_VIDEO_MODE_1600x1200_MONO8,
                  DC1394_VIDEO_MODE_1280x960_MONO16,
                  DC1394_VIDEO_MODE_1600x1200_MONO16,
                  DC1394_VIDEO_MODE_EXIF,
                  DC1394_VIDEO_MODE_FORMAT7_0,
                  DC1394_VIDEO_MODE_FORMAT7_1,
                  DC1394_VIDEO_MODE_FORMAT7_2,
                  DC1394_VIDEO_MODE_FORMAT7_3,
                  DC1394_VIDEO_MODE_FORMAT7_4,
                  DC1394_VIDEO_MODE_FORMAT7_5,
                  DC1394_VIDEO_MODE_FORMAT7_6,
                  DC1394_VIDEO_MODE_FORMAT7_7
                 */



                // // THIS MUST BE CALLED AT SOME POINT
                // err=dc1394_capture_setup(cameras[i],NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
                // DC1394_ERR_CLN_RTN(err,cleanup(),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera");

                // // How to start streaming
                // err=dc1394_video_set_transmission(cameras[i], DC1394_ON);
                // DC1394_ERR_CLN_RTN(err,cleanup(),"Could not start camera iso transmission");

                // // THIS GETS AN FD THAT YOU CAN DO SELECT ON
                // dc1394_capture_get_fileno(camera);

                // // Capturing or something
                // for (i = 0; i < numCameras; i++) {
                // if (dc1394_capture_dequeue(cameras[i], DC1394_CAPTURE_POLICY_WAIT, &frames[i])!=DC1394_SUCCESS)
                //         dc1394_log_error("Failed to capture from camera %d", i);
                // }

                // // Re-enquing frames
                // for (i = 0; i < numCameras; i++) {
                // if (frames[i])
                //         dc1394_capture_enqueue (cameras[i], frames[i]);
                // }
            }

            // TODO Apply our settings
        });

        on<Trigger<Every<225, Per<std::chrono::minutes>>>, Options<Single>>([this](const time_t&) {

            dc1394video_frame_t* frame;
            for(auto& camera : cameras) {

                dc1394_capture_dequeue(camera.second.get(), DC1394_CAPTURE_POLICY_WAIT, &frame);

                uint width = 640;
                uint height = 480;
                std::cout << "Getting image from " << camera.second.get() << std::endl;

                std::vector<Image::Pixel> pixels;

                emit(std::make_unique<Image>(width, height, std::move(pixels)));

                dc1394_capture_enqueue(camera.second.get(), frame);
            }
        });
    }

}
}

