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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_DARWIN_SENSORFILTER_H
#define MODULES_PLATFORM_DARWIN_SENSORFILTER_H

#include <Eigen/Core>
#include <nuclear>

#include "utility/math/filter/UKF.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"

#include "DarwinVirtualLoadSensor.h"
#include "MotionModel.h"
#include "message/motion/KinematicsModels.h"
#include "utility/math/matrix/Rotation3D.h"


namespace module {
namespace platform {
    namespace darwin {

        /**
         * TODO document
         *
         * @author Jake Fountain
         * @author Trent Houliston
         */
        class SensorFilter : public NUClear::Reactor {
        public:
            explicit SensorFilter(std::unique_ptr<NUClear::Environment> environment);

            utility::math::filter::UKF<MotionModel> motionFilter;

            struct Config {
                Config() : battery(), motionFilter(), buttons() {}

                struct Battery {
                    Battery() : chargedVoltage(0.0f), flatVoltage(0.0f) {}
                    float chargedVoltage;
                    float flatVoltage;
                } battery;

                struct MotionFilter {
                    MotionFilter() : velocityDecay(Eigen::Vector3d::Zero()), noise(), initial() {}

                    Eigen::Vector3d velocityDecay;

                    struct Noise {
                        Noise() : measurement(), process() {}
                        struct Measurement {
                            Measurement()
                                : accelerometer(Eigen::Matrix3d::Identity())
                                , accelerometerMagnitude(Eigen::Matrix3d::Identity())
                                , gyroscope(Eigen::Matrix3d::Identity())
                                , footUpWithZ(Eigen::Matrix4d::Identity())
                                , flatFootOdometry(Eigen::Matrix3d::Identity())
                                , flatFootOrientation(Eigen::Matrix4d::Identity()) {}
                            Eigen::Matrix3d accelerometer;
                            Eigen::Matrix3d accelerometerMagnitude;
                            Eigen::Matrix3d gyroscope;
                            Eigen::Matrix4d footUpWithZ;
                            Eigen::Matrix3d flatFootOdometry;
                            Eigen::Matrix4d flatFootOrientation;
                        } measurement;
                        struct Process {
                            Process()
                                : position(Eigen::Vector3d::Ones())
                                , velocity(Eigen::Vector3d::Ones())
                                , rotation(Eigen::Vector4d::Ones())
                                , rotationalVelocity(Eigen::Vector3d::Ones()) {}
                            Eigen::Vector3d position;
                            Eigen::Vector3d velocity;
                            Eigen::Vector4d rotation;
                            Eigen::Vector3d rotationalVelocity;
                        } process;
                    } noise;
                    struct Initial {
                        Initial() : mean(), covariance() {}
                        struct Mean {
                            Mean()
                                : position(Eigen::Vector3d::Ones())
                                , velocity(Eigen::Vector3d::Ones())
                                , rotation(Eigen::Vector4d::Ones())
                                , rotationalVelocity(Eigen::Vector3d::Ones()) {}
                            Eigen::Vector3d position;
                            Eigen::Vector3d velocity;
                            Eigen::Vector4d rotation;
                            Eigen::Vector3d rotationalVelocity;
                        } mean;
                        struct Covariance {
                            Covariance()
                                : position(Eigen::Vector3d::Ones())
                                , velocity(Eigen::Vector3d::Ones())
                                , rotation(Eigen::Vector4d::Ones())
                                , rotationalVelocity(Eigen::Vector3d::Ones()) {}
                            Eigen::Vector3d position;
                            Eigen::Vector3d velocity;
                            Eigen::Vector4d rotation;
                            Eigen::Vector3d rotationalVelocity;
                        } covariance;
                    } initial;
                } motionFilter;

                struct Button {
                    Button() : debounceThreshold(0) {}
                    int debounceThreshold;
                } buttons;
            } config;

        private:
            // Current state of the button pushes
            // used to debounce button presses
            bool leftDown   = false;
            bool middleDown = false;

            // Our sensor for foot down
            DarwinVirtualLoadSensor leftFootDown;
            DarwinVirtualLoadSensor rightFootDown;

            // World to foot in world rotation when the foot landed
            std::array<Eigen::Vector3d, 2> footlanding_rFWw;

            // Foot to world in foot-flat rotation when the foot landed
            std::array<utility::math::matrix::Rotation3D, 2> footlanding_Rfw;

            // World to foot in foot-flat rotation when the foot landed
            std::array<utility::math::matrix::Rotation3D, 2> footlanding_Rwf;
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module
#endif  // MODULES_PLATFORM_DARWIN_SENSORFILTER_H
