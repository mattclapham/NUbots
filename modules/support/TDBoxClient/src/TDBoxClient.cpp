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

#include "TDBoxClient.h"

#include <sstream>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <unistd.h>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "messages/support/Configuration.h"

namespace modules {
namespace support {

    using messages::support::Configuration;

    void TDBoxClient::reconnect() {
        // Open a file descriptor to the target address
        if(fd) {
            close(fd);
        }

        // Open a new socket
        fd = socket(AF_INET, SOCK_STREAM, 0);

        if (fd < 0) {
            std::system_error(errno, std::system_category(), "Failed to open socket for Technical Director Box");
        }

        // Get the information we need to connect
        sockaddr_in serverAddress;
        hostent* h = gethostbyname(host.c_str());

        std::memset(&serverAddress, 0, sizeof(sockaddr_in));
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(port);
        std::memcpy(h->h_addr, &serverAddress.sin_addr.s_addr, h->h_length);

        if (connect(fd, reinterpret_cast<sockaddr*>(&serverAddress), sizeof(serverAddress)) < 0) {
            std::system_error(errno, std::system_category(), "Failed to connect to the Technical Director Box");
        }
    }

    std::string TDBoxClient::nmeaUTCTime() {

        std::time_t t1 = NUClear::clock::to_time_t(NUClear::clock::now());
        std::tm* wtf = gmtime(&t1);
        std::vector<char> timeOutput(7, 0);
        strftime(timeOutput.data(), timeOutput.size(), "%H%M%S", wtf);

        return timeOutput.data();
    }

    void TDBoxClient::sendNMEA(const std::vector<std::string>& messages) {

        std::lock_guard<std::mutex> lock(sendMutex);

        std::stringstream s;

        // Output the start
        s << "$";

        // Output our list
        for(uint i = 0; i < messages.size(); ++i) {
            s << messages[i];

            // If we have more data comma separate
            if(i < messages.size() - 1) {
                s << ",";
            }
        }

        // Checksum and finish
        uint checksum = 0;
        std::string msg = s.str();
        for(uint i = 1; i < msg.size(); ++i) {
            checksum ^= msg[i];
        }
        s << "*";
        s << std::setfill('0') << std::setw(2) << std::hex << checksum;
        s << "\r\n";

        // Rebuild the string with the checksum
        msg = s.str();

        // Write to the socket
        int result = send(fd, msg.data(), msg.size(), MSG_NOSIGNAL);

        // Reconnect
        if(result == -1) {
            reconnect();
        }


    }

    TDBoxClient::TDBoxClient(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Trigger<Configuration<TDBoxClient>>>([this](const Configuration<TDBoxClient>& config) {

            // Get our config
            port = config["port"].as<uint>();
            host = config["host"].as<std::string>();

            reconnect();
        });

        on<Trigger<Every<1, std::chrono::seconds>>>([this](const time_t&) {

            sendNMEA({
                "RXHRT",       // Header
                nmeaUTCTime(), // Time
                "",            // Latitude
                "",            // Latitude Direction
                "",            // Longitude
                "",            // Longitude Direction
                "NCSTL",       // Team ID
                "1",           // Vehicle Mode (1 = rc, 2 = autonomous)
                "SLACKING OFF" // Current task
            });
        });

        on<Trigger<Every<1, std::chrono::seconds>>>([this](const time_t&) {

            sendNMEA({
                "RXSEA",       // Header
                nmeaUTCTime(), // Time
                "NCSTL",       // Team ID
                "",            // Buoy Colour
                "",            // Latitude
                "",            // Latitude Direction
                "",            // Longitude
                "",            // Longitude Direction
                "",            // Pinger depth
            });
        });

        on<Trigger<Every<1, std::chrono::seconds>>>([this](const time_t&) {

            sendNMEA({
                "RXLIT",       // Header
                nmeaUTCTime(), // Time
                "NCSTL",       // Team ID
                ""             // Light Pattern
            });
        });
    }

}
}

