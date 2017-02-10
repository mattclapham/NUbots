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

#include "LUTClassifier.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
    namespace vision {

        using message::input::Image;
        using message::vision::LookUpTable;
        using message::vision::ClassifiedImage;

        using ServoID = utility::input::ServoID;
        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform3D;

        void LUTClassifier::findHorizon(const Image& image, const LookUpTable&, ClassifiedImage& classifiedImage) {

                auto& sensors = *classifiedImage.sensors;

                // Get our transform to world coordinates
                const Rotation3D& Rtw = Transform3D(convert<double, 4, 4>(sensors.world)).rotation();
                const Rotation3D& Rtc = Transform3D(convert<double, 4, 4>(sensors.forwardKinematics.at(ServoID::HEAD_PITCH))).rotation();
                Rotation3D Rcw =  Rtc.i() * Rtw;

                // Rcw = Rotation3D::createRotationZ(-Rcw.yaw()) * Rcw;

                // Coordinate system: 0,0 is the centre of the screen. pos[0] is along the y axis of the
                // camera transform, pos[1] is along the z axis (x points out of the camera)
                auto horizon = utility::motion::kinematics::calculateHorizon(Rcw, FOCAL_LENGTH_PIXELS);
                classifiedImage.horizon.normal = convert<double, 2>(horizon.normal);

                // Move our axis to be at the top left of the screen
                classifiedImage.horizon.distance = -horizon.distanceToPoint({ -double(image.dimensions[0]) * 0.5, -double(image.dimensions[1]) * 0.5 });
        }

    }  // vision
}  // modules
