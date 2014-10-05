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

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

namespace modules {
namespace support {

    using messages::support::Configuration;
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
                message.ParseFromArray(data.data(), data.size());

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
                    message.ParseFromArray(data.data(), data.size());

                    // If it's an image
                    if(message.type() == Message::IMAGE) {

                        // Work out our time to run
                        time_t timeToRun = time_t(std::chrono::milliseconds(message.utc_timestamp())) - offset;

                        // Wait until it's time to display it
                        std::this_thread::sleep_until(timeToRun);

                        // Send it!
                        std::cout << "makin images!" << std::endl;
                    }

                }
            },
            [] {
                // This is a buggy module! recode me!
            }));

            // Add a service thread that just reads for now

            // Open the file

            // Read the next size

            // Read that into a message

            // For the first one get the difference between its timestamp and the current as "offset"

            // Sleep until the next offset

            // Emit while next offset is not 0

            // Add handlers for each type

    }

}
}

