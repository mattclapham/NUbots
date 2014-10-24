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

#ifndef MODULES_ROBOTX_COMMUNICATOR_H
#define MODULES_ROBOTX_COMMUNICATOR_H

#define __USE_SINGLE_PRECISION__

#include <thread>
#include <chrono>
#include <nuclear>
#include <string>
#include <NURobotX/NetworkIO/P2PConnector.hpp>
#include "TcpClientStream.h"
#include "TcpServerStream.h"

namespace modules {
namespace robotx {

    struct StdThreadWaitPolicy
    {
        void wait(uint ms)
        {
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
    };

    class Communicator : public NUClear::Reactor {

        typedef NURobotX::P2PConnector<TcpClientStream, StdThreadWaitPolicy, std::thread> STMConnector;
        typedef NURobotX::P2PConnector<TcpServerStream, StdThreadWaitPolicy, std::thread> RemoteConnector;

    private:
        uint port;
        std::string stm_ip_address;
        std::string stm_port;
        TcpClientStream client_stream;
        TcpServerStream server_stream;
        std::unique_ptr<STMConnector> stm_connector;
        std::unique_ptr<RemoteConnector> remote_connector;
    public:
        /// @brief Called by the powerplant to build and setup the Communicator reactor.
        explicit Communicator(std::unique_ptr<NUClear::Environment> environment);
        static constexpr const char* CONFIGURATION_PATH = "Communicator.yaml";
    };

}
}

#endif
