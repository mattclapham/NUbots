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

#include "BuoyDetector.h"

#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"

#include "utility/math/geometry/Plane.h"

#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacOriginConeModel.h"

#include "utility/math/vision.h"
#include "utility/vision/geometry/screen.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace vision {

    //using messages::input::CameraParameters;
    using messages::input::Sensors;

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::VisionObject;
    using messages::vision::Ball;

    using Plane = utility::math::geometry::Plane<3>;

    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;

    using utility::math::coordinates::cartesianToSpherical;
    using utility::nubugger::graph;

    using messages::support::Configuration;
    using messages::support::FieldDescription;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacOriginConeModel;

    BuoyDetector::BuoyDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<BuoyDetector>>>([this](const Configuration<BuoyDetector>& config) {
            MINIMUM_POINTS_FOR_CONSENSUS = config["ransac"]["minimum_points_for_consensus"].as<uint>();
            CONSENSUS_ERROR_THRESHOLD = config["ransac"]["consensus_error_threshold"].as<double>();
            MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
            MAXIMUM_FITTED_MODELS = config["ransac"]["maximum_fitted_models"].as<uint>();
            MAXIMUM_DISAGREEMENT_RATIO = config["maximum_disagreement_ratio"].as<double>();
            measurement_distance_variance_factor = config["measurement_distance_variance_factor"].as<double>();
            measurement_bearing_variance = config["measurement_bearing_variance"].as<double>();
            measurement_elevation_variance = config["measurement_elevation_variance"].as<double>();
        });

        on<Trigger<Raw<ClassifiedImage<ObjectClass>>>, With<Optional<FieldDescription>>, Options<Single>>("Ball Detector", [this](
            const std::shared_ptr<const ClassifiedImage<ObjectClass>>& rawImage, const std::shared_ptr<const FieldDescription>& field) {
            if (field == nullptr) {
                NUClear::log(__FILE__, ", ", __LINE__, ": FieldDescription Update: support::configuration::SoccerConfig module might not be installed.");
                throw std::runtime_error("FieldDescription Update: support::configuration::SoccerConfig module might not be installed");
            }
            const auto& image = *rawImage;
            // This holds our points that may be a part of the ball
            //std::vector<arma::ivec2> ballPoints;
            const auto& sensors = *image.sensors;

            // Get all the points that could make up the ball
            
            arma::imat ballPoints( image.horizontalSegments.count(ObjectClass::BALL) + image.verticalSegments.count(ObjectClass::BALL), 2);
            uint total = 0;
            
            for(int i = 0; i < 1; ++i) {

                auto segments = i ? image.horizontalSegments.equal_range(ObjectClass::BALL)
                                  : image.verticalSegments.equal_range(ObjectClass::BALL);

                for(auto it = segments.first; it != segments.second; ++it) {

                    auto& segment = it->second;
                    auto& start = segment.start;
                    auto& end = segment.end;

                    bool belowHorizon = image.visualHorizonAtPoint(end[0]) < end[1] || image.visualHorizonAtPoint(start[0]) < start[1];

                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on either side (are on an edge)
                    // Go from an orange to other to orange segment (are interior)

                    if(belowHorizon
                        && segment.subsample == 1
                        && segment.next
                        && (!segment.next->next || segment.next->next->colour != ObjectClass::BALL)) {

                        ballPoints.row(total) = arma::irowvec2({ (end[0]), (end[1]) });
                        ++total;
                    }

                    if(belowHorizon
                        && segment.subsample == 1
                        && segment.previous
                        && (!segment.previous->previous || segment.previous->previous->colour != ObjectClass::BALL)) {

                        ballPoints.row(total) = arma::irowvec2({ (start[0]), (start[1]) });
                        ++total;
                    }
                }
            }
            
            //convert pixels to rays
            arma::mat tmpRays = utility::vision::geometry::bulkPixel2Ray(ballPoints.rows(0,total-1),*(image.image)).t();
            std::vector<arma::vec3> ballRays(tmpRays.n_cols);
            for (int i = 0; i < ballRays.size(); ++i) {
                ballRays[i] = tmpRays.col(i);
            }
            
            
            // Use ransac to find the ball
            auto ransacResults = Ransac<RansacOriginConeModel>::fitModels(ballRays.begin()
                                                                    , ballRays.end()
                                                                    , MINIMUM_POINTS_FOR_CONSENSUS
                                                                    , MAXIMUM_ITERATIONS_PER_FITTING
                                                                    , MAXIMUM_FITTED_MODELS
                                                                    , CONSENSUS_ERROR_THRESHOLD);

            auto balls = std::make_unique<std::vector<Ball>>();
            balls->reserve(ransacResults.size());

            for(auto& result : ransacResults) {

                std::vector<VisionObject::Measurement> measurements;
                measurements.reserve(2);

                // Transform our centre into kinematics coordinates
                /*auto centre = imageToScreen(result.model.centre, image.dimensions);

                // Get the 4 points around our circle
                auto top   = centre + arma::vec2({ 0,  result.model.radius });
                auto base  = centre + arma::vec2({ 0, -result.model.radius });
                auto left  = centre + arma::vec2({  result.model.radius, 0 });
                auto right = centre + arma::vec2({ -result.model.radius, 0 });*/

                

                // Get a unit vector pointing to the centre of the ball
                arma::vec3 ballCentreRay = result.model.centre;
                
                arma::vec3 worldBallCentreRay = image.camToGround * ballCentreRay.submat(0,0,2,2);
                double cameraHeight = image.camToGround(2, 3);
                
                //these are the possible buoy sizes
                std::vector<double> sizes;
                sizes.push_back(0.58);
                
                //calculate distances as (tandist,widthdist) for all sizes
                double groundDist = 1000.0;
                double widthDist = 0.0;
                for (uint i = 0; i < sizes.size; ++i) {
                    double distMultiplier = (cameraHeight- sizes[i]/2.0) / worldBallCentreRay[2];
                    
                    double gd = arma::norm( worldBallCentreRay.rows(0,1) * distMultiplier);
                    double wd = sizes[i]/result.model.radius;
                    
                    if (abs(gd - wd) < abs(groundDist - widthDist)) {
                        groundDist = gd;
                        widthDist = wd;
                    }
                }
                
                arma::mat ballCentreGroundWidthCov = arma::diagmat(arma::vec({
                    measurement_distance_variance_factor * widthDist,
                    measurement_bearing_variance,
                    measurement_elevation_variance }));
                arma::vec3 ballCentreGroundWidth = widthDist*worldBallCentreRay;
                measurements.push_back({ cartesianToSpherical(ballCentreGroundWidth), ballCentreGroundWidthCov});
                // 0.003505351, 0.001961638, 1.68276E-05
                emit(graph("ballCentreGroundWidth measurement", ballCentreGroundWidth(0), ballCentreGroundWidth(1), ballCentreGroundWidth(2)));
                emit(graph("ballCentreGroundWidth measurement (spherical)", measurements.back().position(0), measurements.back().position(1), measurements.back().position(2)));
                
                
                arma::mat ballCentreGroundProjCov = arma::diagmat(arma::vec({
                    measurement_distance_variance_factor * groundDist,
                    measurement_bearing_variance,
                    measurement_elevation_variance }));
                arma::vec3 ballCentreGroundProj = groundDist*worldBallCentreRay;
                measurements.push_back({ cartesianToSpherical(ballCentreGroundProj), ballCentreGroundProjCov});
                // 0.002357231 * 2, 2.20107E-05 * 2, 4.33072E-05 * 2,
                emit(graph("ballCentreGroundProj measurement", ballCentreGroundProj(0), ballCentreGroundProj(1), ballCentreGroundProj(2)));
                emit(graph("ballCentreGroundProj measurement (spherical)", measurements.back().position(0), measurements.back().position(1), measurements.back().position(2)));

                /*
                 *  IF VALID BUILD OUR BALL
                 */
                if(widthDist > cameraHeight / 2.0 && std::abs((ballCentreGroundWidth[0] - ballCentreGroundProj[0]) / ballCentreGroundProj[0]) > MAXIMUM_DISAGREEMENT_RATIO) {
                    Ball b;

                    // On screen visual shape
                    b.circle.radius = result.model.radius;
                    b.circle.centre = result.model.centre;

                    // Angular positions from the camera
                    b.screenAngular = arma::vec2({std::acos(worldBallCentreRay[1]), std::acos(worldBallCentreRay[2])}); //2.0*std::acos(cam.pixelsToTanThetaFactor % ballCentreScreen);
                    b.angularSize = { 2.0*std::acos(result.model.radius), 2.0*std::acos(result.model.radius) };

                    // Move our measurements
                    b.measurements = std::move(measurements);

                    b.sensors = image.sensors;
                    b.classifiedImage = rawImage;
                    balls->push_back(std::move(b));
                }
            }

            for(auto a = balls->begin(); a != balls->end(); ++a) {
                for(auto b = a + 1; b != balls->end();) {

                    // If our balls overlap
                    if(a->circle.distanceToPoint(b->circle.centre) < b->circle.radius) {
                        // Pick the better ball
                        if(a->circle.radius < b->circle.radius) {
                            // Throwout b
                            b = balls->erase(b);
                        }
                        else {
                            a = balls->erase(a);

                            if(a == b) {
                                ++b;
                            }
                        }
                    }
                    else {
                        ++b;
                    }
                }
            }

            emit(std::move(balls));

        });
    }

}
}
