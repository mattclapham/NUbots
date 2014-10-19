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

#include "NBZPlayer.h"

#include "messages/support/Configuration.h"
#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/input/Image.h"

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

namespace modules {
namespace support {

    using messages::support::Configuration;
    using messages::input::Image;
    using messages::support::nubugger::proto::Message;

    NBZPlayer::NBZPlayer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


            on<Trigger<Configuration<NBZPlayer>>>([this](const Configuration<NBZPlayer>& config) {

                std::string path = config["file"].as<std::string>();

                // Setup the file
                input.push(boost::iostreams::gzip_decompressor());
                input.push(boost::iostreams::file_descriptor_source(path, std::ios_base::in | std::ios_base::binary));

                // Read the first 32 bit int to work out the size
                uint32_t size;
                input >> size;

                // Read that much into a string
                std::vector<char> data(size);
                input.read(data.data(), size);

                // Read the message
                Message message;
                message.ParsePartialFromArray(data.data(), data.size());

                offset = NUClear::clock::now() - time_t(std::chrono::milliseconds(message.utc_timestamp()));
            });


            powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask([this] {
                while(true) {

                    // Read the first 32 bit int to work out the size
                    uint32_t size;
                    input >> size;

                    // Read that much into a string
                    std::vector<char> data(size);
                    input.read(data.data(), size);

                    // Read the message
                    Message message;
                    message.ParsePartialFromArray(data.data(), data.size());

                    // If it's an image
                    if(message.type() == Message::IMAGE) {

                        // Work out our time to run
                        time_t timeToRun = time_t(std::chrono::milliseconds(message.utc_timestamp())) - offset;

                        // Wait until it's time to display it
                        std::this_thread::sleep_until(timeToRun);

                        // Send it!

                        // Get the width and height
                        int width = message.image().dimensions().y();
                        int height = message.image().dimensions().x();
                        const std::string& source = message.image().data();

                        // Get the image data
                        std::vector<uint8_t> data(source.begin(), source.end());

                        static int cam = 0;
                        cam = !cam;
                        switch(cam) {
                            case 0: {
                                auto image = std::make_unique<Image<0>>();
                                image->timestamp = NUClear::clock::now();
                                image->format =  Image<0>::SourceFormat::BGGR;
                                image->dimensions = { 1280, 960 };
                                image->cameraToGround = arma::eye(4,4);
                                image->source = std::move(data);

                                // The lens! (hardcoded :P)
                                image->lens.type = Image<0>::Lens::Type::RADIAL;
                                image->lens.parameters.radial.fov = M_PI;
                                image->lens.parameters.radial.pitch = 0.0025;
                                image->lens.parameters.radial.centre[0] = 640;
                                image->lens.parameters.radial.centre[1] = 480;

                                emit(std::move(image));

                            } break;

                            case 1: {
                                auto image = std::make_unique<Image<1>>();
                                image->timestamp = NUClear::clock::now();
                                image->format =  Image<1>::SourceFormat::BGGR;
                                image->dimensions = { 1280, 960 };
                                image->cameraToGround = arma::eye(4,4);
                                image->source = std::move(data);

                                // The lens! (hardcoded :P)
                                image->lens.type = Image<1>::Lens::Type::RADIAL;
                                image->lens.parameters.radial.fov = M_PI;
                                image->lens.parameters.radial.pitch = 0.0025;
                                image->lens.parameters.radial.centre[0] = 640;
                                image->lens.parameters.radial.centre[1] = 480;

                                emit(std::move(image));

                            } break;
                        }


                    }

                }
            },
            [] {
                // This is a buggy module! recode me!
            }));

    }

}
}

