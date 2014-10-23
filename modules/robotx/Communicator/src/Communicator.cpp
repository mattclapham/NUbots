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

#include "Communicator.h"
#include "messages/support/Configuration.h"
#include <NURobotX/NetworkIO/MessageType.h>
#include <NURobotX/NetworkIO/SerializationTraits.hpp>
#include <NURobotX/Util/Operators.hpp>
#include <NURobotX/Data/VehicleState.h>
#include "messages/input/Sensors.h"
#include "messages/input/GPS.h"
#include <eigen3/Eigen/Core>

namespace modules {
namespace robotx {

    using messages::support::Configuration;
    using messages::input::Sensors;
    using messages::input::GPS;
    using namespace NURobotX;

    Communicator::Communicator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<Communicator>>>([this] (const Configuration<Communicator>& file) {
            port = file.config["listenPort"].as<uint>();
            stm_ip_address = file.config["stmIPAddress"].as<std::string>();
            stm_port = file.config["stmPort"].as<std::string>();

            client_stream.init(stm_ip_address, stm_port);
            server_stream.init(port);

            stm_connector.reset(new STMConnector(client_stream));
            remote_connector.reset(new RemoteConnector(server_stream));

            remote_connector->registerMessageHandler<MessageType::ACTUATORS_ON>([this] (bool& on) {
                try
                {
                    stm_connector->send<MessageType::ACTUATORS_ON>(on);
                }
                catch(std::exception& ex)
                {
                    NUClear::log(static_cast<std::string>("ACTUATORS_ON ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::STATE_ESTIMATOR_PARAMETERS>([this] (StateEstimatorParameters& params) {
                try
                {
                    stm_connector->send<MessageType::STATE_ESTIMATOR_PARAMETERS>(params);
                }
                catch(std::exception& ex)
                {
                    NUClear::log(static_cast<std::string>("STATE_ESTIMATOR_PARAMETERS ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::INITIAL_CONDITIONS>([this] (InitialConditions& initial_conditions) {
                try
                {
                    stm_connector->send<MessageType::INITIAL_CONDITIONS>(initial_conditions);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("INITIAL_CONDITIONS ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::HEADING_PID>([this] (PID& pid) {
                try
                {
                    stm_connector->send<MessageType::HEADING_PID>(pid);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("HEADING_PID ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::VELOCITY_PID>([this] (PID& pid) {
                try
                {
                    stm_connector->send<MessageType::VELOCITY_PID>(pid);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("VELOCITY_PID ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::LEFT_ACTUATOR_PID>([this] (PID& pid) {
                try
                {
                    stm_connector->send<MessageType::LEFT_ACTUATOR_PID>(pid);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("LEFT_ACTUATOR_PID ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::RIGHT_ACTUATOR_PID>([this] (PID& pid) {
                try
                {
                    stm_connector->send<MessageType::RIGHT_ACTUATOR_PID>(pid);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("RIGHT_ACTUATOR_PID ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::CONTROL_MODE>([this] (ControlMode& mode) {
                try
                {
                    stm_connector->send<MessageType::CONTROL_MODE>(mode);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("CONTROL_MODE ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::ACTUATOR_COMMAND>([this] (ActuatorCommand& command) {
                try
                {
                    stm_connector->send<MessageType::ACTUATOR_COMMAND>(command);
                }
                catch(std::exception& ex)
                {
                    log(static_cast<std::string>("ACTUATOR_COMMAND ERROR: ") + ex.what());
                }
            });

            remote_connector->registerMessageHandler<MessageType::DATA_LOGGING_ON>([this] (bool& log) {
                try
                {
                    stm_connector->send<MessageType::DATA_LOGGING_ON>(log);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("DATA_LOGGING_ON ERROR: ") + ex.what());
                }
            });

            stm_connector->registerMessageHandler<MessageType::ACTUATOR_STATUS>([this] (ActuatorStatus& status) {
                try
                {
                    remote_connector->send<MessageType::ACTUATOR_STATUS>(status);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("ACTUATOR_STATUS ERROR: ") + ex.what());
                }
            });

            stm_connector->registerMessageHandler<MessageType::TUNING_PARAMETERS>([this] (TuningParameters& params) {
                try
                {
                    remote_connector->send<MessageType::TUNING_PARAMETERS>(params);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("TUNING_PARAMETERS ERROR: ") + ex.what());
                }
            });

            stm_connector->registerMessageHandler<MessageType::VEHICLE_GPS>([this] (VehicleGPS& vehicle_gps) {
                auto gps = std::make_unique<GPS>();
                gps->lattitude = vehicle_gps.lattitude;
                gps->longitude = vehicle_gps.longitude;
                gps->altitude = vehicle_gps.altitude;
                emit(std::move(gps));
            });

            stm_connector->registerMessageHandler<MessageType::STATE_ESTIMATE>([this] (State& state) {
                try
                {
                    remote_connector->send<MessageType::STATE_ESTIMATE>(state);
                }
                catch(std::exception& ex)
                {
                     NUClear::log(static_cast<std::string>("STATE_ESTIMATE ERROR: ") + ex.what());
                }

                auto sensors = std::make_unique<Sensors>();

                Vector3s thetanb = state.state.thetanb();

                Eigen::Matrix3f Tlr; //Transform for RHS to LHS coordinates;
                Tlr << 1, 0, 0,
                           0, 1, 0,
                           0, 0,-1;
                Eigen::Matrix3d orientation((Tlr*NURobotX::Operators::Rotation(thetanb)).cast<double>());
                //sensors->orientation = arma::mat33(orientation.data(),3,3);

                sensors->vehicleStateMean = arma::fvec(state.state.mean().data(),15, 1);
                sensors->vehicleStateCovariance = arma::fvec(state.state.covariance().data(),15, 15);
                sensors->vehicleStateTImestamp = state.state.time_stamp;

                emit(std::move(sensors));

            });

        });

        // Ensure connection
        on<Trigger<Every<1, Per<std::chrono::seconds>>>>([this] (const time_t&) {
            if (remote_connector && !remote_connector->isConnected()) {
                remote_connector->connect();
            }
            if (stm_connector && !stm_connector->isConnected()) {
                stm_connector->connect();
            }
        });
    }
}
}

