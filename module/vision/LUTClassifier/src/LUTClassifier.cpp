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

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/support/SaveConfiguration.h"
#include "message/vision/LookUpTable.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/LookUpTable.h"

namespace module {
    namespace vision {

        using extension::Configuration;

        using message::input::Image;
        using message::input::Sensors;
        using message::input::CameraParameters;
        using message::vision::LookUpTable;
        using message::vision::SaveLookUpTable;
        using message::vision::ClassifiedImage;
        using message::support::SaveConfiguration;
        using utility::support::Expression;

        //using ServoID = utility::input::ServoID;
        using Colour  = utility::vision::Colour;
        using Pixel   = utility::vision::Pixel;

        void LUTClassifier::insertSegments(ClassifiedImage& image, std::vector<ClassifiedImage::Segment>& segments, bool vertical) {

            int32_t previous = -1;

            auto& target = vertical ? image.verticalSegments : image.horizontalSegments;

            for (auto& segment : segments) {

                // Move in the data
                auto current = target.insert(target.end(), std::move(segment));

                // Link up the results
                current->previous = previous;

                if (previous > -1)
                {
                    target[previous].next = std::distance(target.begin(), current);
                }

                // Get ready for our next one
                previous = std::distance(target.begin(), current);
            }
        }

        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , quex(new QuexClassifier)
            , greenCentroid(Eigen::Vector3f::Zero())
            , LUT_PATH("")
            , LUT_HOST("") {

            on<Configuration>("LookUpTable.yaml").then([this] (const Configuration& config) {

                // Load our LUT
                auto lut = std::make_unique<LookUpTable>(config.config.as<LookUpTable>());

                // Calculate our green centroid for ball detection
                Eigen::Vector3f greenCentroid(0, 0, 0);
                uint nPoints = 0;

                // Loop through every voxel in the lut
                for(uint x = 0; x < uint(1 << lut->bits_y); ++x) {
                    for (uint y = 0; y < uint(1 << lut->bits_cb); ++y) {
                        for (uint z = 0; z < uint(1 << lut->bits_cr); ++z) {

                            // Get our voxel
                            uint index = (((x << lut->bits_cr) | y) << lut->bits_cb) | z;
                            char c = lut->table[index];

                            // If this is a field voxel
                            if (c == static_cast<char>(Colour::GREEN)) {
                                // Get our LUT pixel for this index
                                Pixel p = utility::vision::getPixelFromIndex(*lut, index);

                                ++nPoints;
                                greenCentroid += Eigen::Vector3f({ float(p.components.y), float(p.components.cb), float(p.components.cr) });
                            }
                        }
                    }
                }
                greenCentroid /= float(nPoints);
                greenCentroid[0] *= 2.0;
                this->greenCentroid = greenCentroid;

                LUT_PATH = config.fileName;
                LUT_HOST = config.hostname;

                emit(std::move(lut));
            });

            on<Trigger<SaveLookUpTable>, With<LookUpTable>>().then([this] (const LookUpTable& lut) {
                std::ofstream yaml("config/" + LUT_HOST + "/" + LUT_PATH, std::ios::trunc | std::ios::out);
                yaml << YAML::convert<LookUpTable>::encode(lut);
                yaml.close();
            });

            // Trigger the same function when either update
            on<Configuration, Trigger<CameraParameters>>("LUTClassifier.yaml").then([this] (const Configuration& config, const CameraParameters& cam) {

                // Visual horizon detector
                VISUAL_HORIZON_SPACING = cam.focalLengthPixels * tan(config["visual_horizon"]["spacing"].as<double>());
                VISUAL_HORIZON_BUFFER = cam.focalLengthPixels * tan(config["visual_horizon"]["horizon_buffer"].as<double>());
                VISUAL_HORIZON_SUBSAMPLING = std::max(1, int(cam.focalLengthPixels * tan(config["visual_horizon"]["subsampling"].as<double>())));
                VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE = cam.focalLengthPixels * tan(config["visual_horizon"]["minimum_segment_size"].as<double>());

                // Goal detector
                GOAL_LINE_SPACING = cam.focalLengthPixels * tan(config["goals"]["spacing"].as<double>());
                GOAL_SUBSAMPLING = std::max(1, int(cam.focalLengthPixels * tan(config["goals"]["subsampling"].as<double>())));
                GOAL_RANSAC_MINIMUM_POINTS_FOR_CONSENSUS = config["goals"]["ransac"]["minimum_points_for_consensus"].as<uint>();
                GOAL_RANSAC_MAXIMUM_ITERATIONS_PER_FITTING = config["goals"]["ransac"]["maximum_iterations_per_fitting"].as<uint>();
                GOAL_RANSAC_MAXIMUM_FITTED_MODELS = config["goals"]["ransac"]["maximum_fitted_models"].as<uint>();
                GOAL_RANSAC_CONSENSUS_ERROR_THRESHOLD = config["goals"]["ransac"]["consensus_error_threshold"].as<double>();
                GOAL_MINIMUM_RANSAC_SEGMENT_SIZE = std::max(1, int(cam.focalLengthPixels * tan(config["goals"]["minimum_ransac_segment_size"].as<double>())));
                GOAL_MAX_HORIZON_ANGLE = std::cos(config["goals"]["max_horizon_angle"].as<Expression>());
                GOAL_RANSAC_CONSENSUS_ERROR_THRESHOLD = config["goals"]["ransac"]["consensus_error_threshold"].as<double>();
                GOAL_LINE_DENSITY = config["goals"]["line_density"].as<int>();
                GOAL_HORIZONTAL_EXTENSION_SCALE = config["goals"]["horizontal_extension_scale"].as<double>();
                GOAL_VERTICAL_EXTENSION_SCALE = config["goals"]["vertical_extension_scale"].as<double>();
                GOAL_WIDTH_HEIGHT_RATIO = config["goals"]["width_height_ratio"].as<double>();
                GOAL_LINE_INTERSECTIONS = config["goals"]["line_intersections"].as<uint>();

                // Ball Detector
                BALL_MINIMUM_INTERSECTIONS_COARSE = config["ball"]["intersections_coarse"].as<double>();
                BALL_MINIMUM_INTERSECTIONS_FINE = config["ball"]["intersections_fine"].as<double>();
                BALL_SEARCH_CIRCLE_SCALE = config["ball"]["search_circle_scale"].as<double>();
                BALL_MAXIMUM_VERTICAL_CLUSTER_SPACING = std::max(1, int(cam.focalLengthPixels * tan(config["ball"]["maximum_vertical_cluster_spacing"].as<double>())));
                BALL_HORIZONTAL_SUBSAMPLE_FACTOR = config["ball"]["horizontal_subsample_factor"].as<double>();

                MAXIMUM_LIGHTNING_BOLT_LENGTH = config["ball"]["maximum_lighting_bolt_length"].as<int>();
                MINIMUM_LIGHTNING_BOLT_STRENGTH = config["ball"]["minimum_lighting_bolt_strength"].as<double>();

                // Camera settings
                ALPHA = cam.pixelsToTanThetaFactor[1];
                FOCAL_LENGTH_PIXELS = cam.focalLengthPixels;
            });

            on<Trigger<Image>
            , With<LookUpTable>
            , With<Sensors>
            , Single
            , Priority::LOW>().then("Classify Image", [this] (const Image& rawImage
                      , const LookUpTable& lut
                      , const Sensors& sensors) {

                //TODO
                // if(std::fabs(sensors.servo[ServoID::HEAD_PITCH].currentVelocity) + std::fabs(sensors.servo[ServoID::HEAD_YAW].currentVelocity) > threshold)

                // Our classified image
                auto classifiedImage = std::make_unique<ClassifiedImage>();

                // Set our width and height
                classifiedImage->dimensions = rawImage.dimensions;

                // Attach our sensors
                // std::cout << "sensor-vision latency = " << std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now() - sensors->timestamp).count() << std::endl;
                classifiedImage->sensors = const_cast<Sensors*>(&sensors)->shared_from_this();

                // Attach the image
                classifiedImage->image = const_cast<Image*>(&rawImage)->shared_from_this();

                // Find our horizon
                findHorizon(rawImage, lut, *classifiedImage);

                // Find our visual horizon
                findVisualHorizon(rawImage, lut, *classifiedImage);

                // Find our goals
                findGoals(rawImage, lut, *classifiedImage);

                // Enhance our goals
                enhanceGoals(rawImage, lut, *classifiedImage);

                // Find our ball (also helps with the bottom of goals)
                findBall(rawImage, lut, *classifiedImage);

                // Enhance our ball
                enhanceBall(rawImage, lut, *classifiedImage);

                // Emit our classified image
                emit(std::move(classifiedImage));
            });

        }
    }  // vision
}  // modules
