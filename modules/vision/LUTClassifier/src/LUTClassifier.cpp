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

#include "messages/input/Image.h"
#include "messages/input/Sensors.h"
#include "messages/vision/LookUpTable.h"
#include "messages/support/Configuration.h"

#include "QuexClassifier.h"

#include "Lexer.hpp"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::ServoID;
        using messages::input::Sensors;
        using messages::vision::LookUpTable;
        using messages::vision::SaveLookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        using messages::support::Configuration;
        using messages::support::SaveConfiguration;

        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), quex(new QuexClassifier) {

            on<Trigger<Configuration<LUTLocation>>>([this](const Configuration<LUTLocation>& config) {
                emit(std::make_unique<LookUpTable>(config.config.as<LookUpTable>()));
            });

            on<Trigger<SaveLookUpTable>, With<LookUpTable>>([this](const SaveLookUpTable&, const LookUpTable& lut) {
                emit(std::make_unique<SaveConfiguration>(SaveConfiguration{ LUTLocation::CONFIGURATION_PATH, YAML::Node(lut) }));
            });

            // Trigger the same function when either update
            on<Trigger<Configuration<LUTClassifier>>>([this] (const Configuration<LUTClassifier>& config) {

                struct Hack {
                    double focalLengthPixels = 1;
                    double pixelsToTanThetaFactor[2] = { 1, 1 };
                } cam;

                // Visual horizon detector
                VISUAL_HORIZON_SPACING = cam.focalLengthPixels * tan(config["visual_horizon"]["spacing"].as<double>());
                VISUAL_HORIZON_BUFFER = cam.focalLengthPixels * tan(config["visual_horizon"]["horizon_buffer"].as<double>());
                VISUAL_HORIZON_SUBSAMPLING = std::max(1, int(cam.focalLengthPixels * tan(config["visual_horizon"]["subsampling"].as<double>())));
                VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE = cam.focalLengthPixels * tan(config["visual_horizon"]["minimum_segment_size"].as<double>());

                // Goal detector
                GOAL_LINE_SPACING = cam.focalLengthPixels * tan(config["goals"]["spacing"].as<double>());
                GOAL_SUBSAMPLING = std::max(1, int(cam.focalLengthPixels * tan(config["goals"]["subsampling"].as<double>())));
                GOAL_EXTENSION_SCALE = config["goals"]["extension_scale"].as<double>() / 2;
                GOAL_LINE_DENSITY = config["goals"]["line_density"].as<int>();

                // Ball Detector
                BALL_MINIMUM_INTERSECTIONS_COARSE = config["ball"]["intersections_coarse"].as<double>();
                BALL_MINIMUM_INTERSECTIONS_FINE = config["ball"]["intersections_fine"].as<double>();
                BALL_SEARCH_CIRCLE_SCALE = config["ball"]["search_circle_scale"].as<double>();
                BALL_MAXIMUM_VERTICAL_CLUSTER_SPACING = std::max(1, int(cam.focalLengthPixels * tan(config["ball"]["maximum_vertical_cluster_spacing"].as<double>())));
                BALL_HORIZONTAL_SUBSAMPLE_FACTOR = config["ball"]["horizontal_subsample_factor"].as<double>();

                // Camera settings
                ALPHA = cam.pixelsToTanThetaFactor[1];
                FOCAL_LENGTH_PIXELS = cam.focalLengthPixels;
            });

            on<Trigger<Raw<Image<0>>>, With<LookUpTable>, With<Raw<Sensors>>, Options<Single>>("Classify Image", [this](
                const std::shared_ptr<const Image<0>>& rawImage, const LookUpTable& lut, const std::shared_ptr<const Sensors>& sensors) {

                const auto& image = *rawImage;

                // Our classified image
                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass, 0>>();

                // Set our width and height
                classifiedImage->dimensions = { image.width(), image.height() };

                // Attach our sensors
                classifiedImage->sensors = sensors;

                // Attach the image
                classifiedImage->image = rawImage;

                // Find our horizon
                findHorizon(image, lut, *classifiedImage);

                // Find our visual horizon
                findVisualHorizon(image, lut, *classifiedImage);

                // Find our goals
                findGoals(image, lut, *classifiedImage);

                // Enhance our goals
                enhanceGoals(image, lut, *classifiedImage);

                // Find our ball (also helps with the bottom of goals)
                findBall(image, lut, *classifiedImage);

                // Enhance our ball
                enhanceBall(image, lut, *classifiedImage);

                // Emit our classified image
                emit(std::move(classifiedImage));
            });

            on<Trigger<Raw<Image<1>>>, With<LookUpTable>, With<Raw<Sensors>>, Options<Single>>("Classify Image", [this](
                const std::shared_ptr<const Image<1>>& rawImage, const LookUpTable& lut, const std::shared_ptr<const Sensors>& sensors) {

                const auto& image = *rawImage;

                // Our classified image
                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass, 1>>();

                // Set our width and height
                classifiedImage->dimensions = { image.width(), image.height() };

                // Attach our sensors
                classifiedImage->sensors = sensors;

                // Attach the image
                classifiedImage->image = rawImage;

                // Find our horizon
                findHorizon(image, lut, *classifiedImage);

                // Find our visual horizon
                findVisualHorizon(image, lut, *classifiedImage);

                // Find our goals
                findGoals(image, lut, *classifiedImage);

                // Enhance our goals
                enhanceGoals(image, lut, *classifiedImage);

                // Find our ball (also helps with the bottom of goals)
                findBall(image, lut, *classifiedImage);

                // Enhance our ball
                enhanceBall(image, lut, *classifiedImage);

                // Emit our classified image
                emit(std::move(classifiedImage));
            });

            on<Trigger<Raw<Image<2>>>, With<LookUpTable>, With<Raw<Sensors>>, Options<Single>>("Classify Image", [this](
                const std::shared_ptr<const Image<2>>& rawImage, const LookUpTable& lut, const std::shared_ptr<const Sensors>& sensors) {

                const auto& image = *rawImage;

                // Our classified image
                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass, 2>>();

                // Set our width and height
                classifiedImage->dimensions = { image.width(), image.height() };

                // Attach our sensors
                classifiedImage->sensors = sensors;

                // Attach the image
                classifiedImage->image = rawImage;

                // Find our horizon
                findHorizon(image, lut, *classifiedImage);

                // Find our visual horizon
                findVisualHorizon(image, lut, *classifiedImage);

                // Find our goals
                findGoals(image, lut, *classifiedImage);

                // Enhance our goals
                enhanceGoals(image, lut, *classifiedImage);

                // Find our ball (also helps with the bottom of goals)
                findBall(image, lut, *classifiedImage);

                // Enhance our ball
                enhanceBall(image, lut, *classifiedImage);

                // Emit our classified image
                emit(std::move(classifiedImage));
            });

            on<Trigger<Raw<Image<3>>>, With<LookUpTable>, With<Raw<Sensors>>, Options<Single>>("Classify Image", [this](
                const std::shared_ptr<const Image<3>>& rawImage, const LookUpTable& lut, const std::shared_ptr<const Sensors>& sensors) {

                const auto& image = *rawImage;

                // Our classified image
                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass, 3>>();

                // Set our width and height
                classifiedImage->dimensions = { image.width(), image.height() };

                // Attach our sensors
                classifiedImage->sensors = sensors;

                // Attach the image
                classifiedImage->image = rawImage;

                // Find our horizon
                findHorizon(image, lut, *classifiedImage);

                // Find our visual horizon
                findVisualHorizon(image, lut, *classifiedImage);

                // Find our goals
                findGoals(image, lut, *classifiedImage);

                // Enhance our goals
                enhanceGoals(image, lut, *classifiedImage);

                // Find our ball (also helps with the bottom of goals)
                findBall(image, lut, *classifiedImage);

                // Enhance our ball
                enhanceBall(image, lut, *classifiedImage);

                // Emit our classified image
                emit(std::move(classifiedImage));
            });

        }

        LUTClassifier::~LUTClassifier() {
            // TODO work out how to fix pimpl and fix it damnit!!
            delete quex;
        }

    }  // vision
}  // modules

#include "HorizonFinder.ipp"
#include "VisualHorizonFinder.ipp"
#include "GoalFinder.ipp"
#include "GoalEnhancer.ipp"
#include "BallFinder.ipp"
#include "BallEnhancer.ipp"