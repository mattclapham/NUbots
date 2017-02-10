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

#include "utility/math/vision.h"
#include "utility/math/geometry/Line.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
    namespace vision {

        using message::input::Image;
        using message::vision::LookUpTable;
        using message::vision::ClassifiedImage;

        using utility::math::geometry::Line;
        using utility::math::vision::getGroundPointFromScreen;
        using utility::math::vision::projectWorldPointToScreen;
        using utility::math::vision::imageToScreen;
        using utility::nubugger::drawVisionLines;
        using Colour = utility::vision::Colour;
        using FOURCC = utility::vision::FOURCC;
        using Pixel  = utility::vision::Pixel;

        std::pair<float, Eigen::Vector2i> fieldEdgeDirection(const Eigen::Vector2i& base, const Image& image, const Eigen::Vector3f& greenCentroid) {

            // Get our relevant pixels
            //TODO:bounds check
            std::array<Pixel, 24> pixels {
                getPixel(base[0] - 2, base[1] - 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 2, base[1] - 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 2, base[1] + 0, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 2, base[1] + 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 2, base[1] + 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),

                getPixel(base[0] - 1, base[1] - 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 1, base[1] - 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 1, base[1] + 0, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 1, base[1] + 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] - 1, base[1] + 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),

                getPixel(base[0] + 0, base[1] - 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 0, base[1] - 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 0, base[1] + 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 0, base[1] + 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),

                getPixel(base[0] + 1, base[1] - 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 1, base[1] - 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 1, base[1] + 0, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 1, base[1] + 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 1, base[1] + 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),

                getPixel(base[0] + 2, base[1] - 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 2, base[1] - 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 2, base[1] + 0, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 2, base[1] + 1, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)),
                getPixel(base[0] + 2, base[1] + 2, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format))
            };

            // Find out how green each pixel is!
            std::array<float, 24> greenness;
            for(int i = 0; i < int(greenness.size()); ++i) {
                greenness[i] = (greenCentroid - Eigen::Vector3f({ float(pixels[i].components.y * 2), float(pixels[i].components.cb), float(pixels[i].components.cr) })).norm();
            }

            constexpr float M_1_SQRT5 = 0.4472135955;
            constexpr float M_2_SQRT5 = 0.894427191;
            constexpr float M_SQRT2_2 = M_SQRT2 * 0.5;

            Eigen::Vector2f greenDirection = (Eigen::Vector2f({-M_SQRT2_2, -M_SQRT2_2}) * greenness[0])
                                           + (Eigen::Vector2f({-M_2_SQRT5, -M_1_SQRT5}) * greenness[1])
                                           + (Eigen::Vector2f({-1        , +0})         * greenness[2])
                                           + (Eigen::Vector2f({-M_2_SQRT5, +M_1_SQRT5}) * greenness[3])
                                           + (Eigen::Vector2f({-M_SQRT2_2, +M_SQRT2_2}) * greenness[4])
                                           + (Eigen::Vector2f({-M_1_SQRT5, -M_2_SQRT5}) * greenness[5])
                                           + (Eigen::Vector2f({-M_SQRT2_2, -M_SQRT2_2}) * greenness[6])
                                           + (Eigen::Vector2f({-1        , +0})         * greenness[7])
                                           + (Eigen::Vector2f({-M_SQRT2_2, +M_SQRT2_2}) * greenness[8])
                                           + (Eigen::Vector2f({-M_1_SQRT5, +M_2_SQRT5}) * greenness[9])
                                           + (Eigen::Vector2f({+0        , -1})         * greenness[10])
                                           + (Eigen::Vector2f({+0        , -1})         * greenness[11])
                                           + (Eigen::Vector2f({+0        , +1})         * greenness[12])
                                           + (Eigen::Vector2f({+0        , +1})         * greenness[13])
                                           + (Eigen::Vector2f({+M_1_SQRT5, -M_2_SQRT5}) * greenness[14])
                                           + (Eigen::Vector2f({+M_SQRT2_2, -M_SQRT2_2}) * greenness[15])
                                           + (Eigen::Vector2f({+1        , +0})         * greenness[16])
                                           + (Eigen::Vector2f({+M_SQRT2_2, +M_SQRT2_2}) * greenness[17])
                                           + (Eigen::Vector2f({+M_1_SQRT5, +M_2_SQRT5}) * greenness[14])
                                           + (Eigen::Vector2f({+M_SQRT2_2, -M_SQRT2_2}) * greenness[19])
                                           + (Eigen::Vector2f({+M_2_SQRT5, -M_1_SQRT5}) * greenness[20])
                                           + (Eigen::Vector2f({+1        , +0})         * greenness[21])
                                           + (Eigen::Vector2f({+M_2_SQRT5, +M_1_SQRT5}) * greenness[22])
                                           + (Eigen::Vector2f({+M_SQRT2_2, +M_SQRT2_2}) * greenness[23]);

            // How strong is our greenness movement?
            double strength = greenDirection.norm();

            // Normalise our direction
            greenDirection /= strength;

            strength /= greenness.size();
            Eigen::Vector2i greenNormal({ -int(std::round(greenDirection[1])), int(std::round(greenDirection[0])) });

            return std::make_pair(strength, greenNormal);
        }

        void LUTClassifier::enhanceBall(const Image& image, const LookUpTable& lut, ClassifiedImage& classifiedImage) {

            // Loop through all of our possible ball segments
            std::vector<Eigen::Vector2i> points;
            // NUClear::log("hSegments size = ", std::distance(hSegments.first,hSegments.second));
            for(auto it = classifiedImage.horizontalSegments.begin(); it != classifiedImage.horizontalSegments.end(); ++it) {

                // We throw out points if they:
                // Have both edges above the green horizon
                // Are too small
                if((it->segmentClass == ClassifiedImage::SegmentClass::BALL) &&
                   (utility::vision::visualHorizonAtPoint(classifiedImage, it->start[0]) <= it->start[1] ||
                    utility::vision::visualHorizonAtPoint(classifiedImage, it->end[0])   <= it->end[1])
                    && it->length > 1) 
                {
                    points.push_back(it->midpoint);
                }
            }

            // std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> debug; // DEBUG LINE
            std::vector<Eigen::Vector2i> edges;

            // For each of these points move upward until we find a strong transition to green
            for(auto& point : points) {
                Line horizon(convert<double, 2>(classifiedImage.horizon.normal), classifiedImage.horizon.distance);
                int minY = int(std::max(3.0, horizon.y(point[0])));
                for(int y = point[1]; y > minY; --y) {

                    auto colour = utility::vision::getPixelColour(lut, getPixel(point[0], y, image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)));

                    if (colour == Colour::GREEN) {
                        auto p = Eigen::Vector2i( point[0], y - 1 );
                        edges.push_back(p);
                        classifiedImage.ballSeedPoints[0].points.push_back(p);
                        // debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1}))); // DEBUG LINE
                        break;
                    }
                }
            }

            // For each of these points move leftward until we find a strong transition to green
            for(auto& point : points) {

                for(int x = point[0]; x > 3; --x) {

                    auto colour = utility::vision::getPixelColour(lut, getPixel(x, point[1], image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)));

                    if (colour == Colour::GREEN) {
                        auto p = Eigen::Vector2i( x + 1, point[1] );
                        edges.push_back(p);
                        classifiedImage.ballSeedPoints[1].points.push_back(p);
                        // debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1}))); // DEBUG LINE
                        break;
                    }
                }
            }

            // For each of these points move rightward until we find a strong transition to green
            for(auto& point : points) {

                for(int x = point[0]; x < int(image.dimensions[0]) - 3; ++x) {

                    auto colour = utility::vision::getPixelColour(lut, getPixel(x, point[1], image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)));

                    if (colour == Colour::GREEN) {
                        auto p = Eigen::Vector2i( x - 1, point[1] );
                        edges.push_back(p);
                        classifiedImage.ballSeedPoints[2].points.push_back(p);
                        // debug.push_back(std::make_tuple(point, edges.back(), arma::vec4({0,1,1,1}))); // DEBUG LINE
                        break;
                    }
                }
            }

            // While we still have edges
            auto setComparator = [] (const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
                return a[0] == b[0] ? a[1] < b[1] : a[0] < b[0];
            };
            std::set<Eigen::Vector2i, decltype(setComparator)> pSet(setComparator);

            for(auto& edge : edges) {

                // The seed points are also edges
                pSet.insert(edge);

                // Go clockwise
                Eigen::Vector2i point = edge;
                for(int i = 0; i < MAXIMUM_LIGHTNING_BOLT_LENGTH; ++i) {

                    // Break if we hit the edge of the screen
                    if(point[0] < 4 || point[0] > (int(image.dimensions[0]) - 4) || point[1] < 4 || point[1] > (int(image.dimensions[1]) - 4)) {
                        break;
                    }

                    // std::tuple<arma::ivec2, arma::ivec2, arma::vec4> d; // DEBUG LINE
                    // std::get<0>(d) = point; // DEBUG LINE

                    float strength;
                    Eigen::Vector2i direction;
                    std::tie(strength, direction) = fieldEdgeDirection(point, image, greenCentroid);

                    // If our strength get's too low then stop
                    if(strength < MINIMUM_LIGHTNING_BOLT_STRENGTH) {
                        break;
                    }

                    point += direction;

                    bool isNew;
                    std::tie(std::ignore, isNew) = pSet.insert(point);

                    if(!isNew) {
                        break;
                    }

                    // std::get<1>(d)  = point; // DEBUG LINE

                    // float r = (strength / 30); // DEBUG LINE
                    // float b = 1 - (strength / 30); // DEBUG LINE
                    // std::get<2>(d)  = arma::vec4({r,0,b,1}); // DEBUG LINE
                    // debug.push_back(d); // DEBUG LINE
                }

                // Go Anticlockwise
                point = edge;
                for(int i = 0; i < MAXIMUM_LIGHTNING_BOLT_LENGTH; ++i) {

                    // Break if we hit the edge of the screen
                    if(point[0] < 4 || point[0] > (int(image.dimensions[0]) - 4) || point[1] < 4 || point[1] > (int(image.dimensions[1]) - 4)) {
                        break;
                    }

                    // std::tuple<arma::ivec2, arma::ivec2, arma::vec4> d; // DEBUG LINE
                    // std::get<0>(d) = point; // DEBUG LINE

                    float strength;
                    Eigen::Vector2i direction;
                    std::tie(strength, direction) = fieldEdgeDirection(point, image, greenCentroid);

                    // If our strength get's too low then stop
                    if(strength < MINIMUM_LIGHTNING_BOLT_STRENGTH) {
                        break;
                    }

                    point -= direction;

                    bool isNew;
                    std::tie(std::ignore, isNew) = pSet.insert(point);

                    if(!isNew) {
                        break;
                    }

                    // std::get<1>(d)  = point; // DEBUG LINE

                    // float r = (strength / 30); // DEBUG LINE
                    // float b = 1 - (strength / 30); // DEBUG LINE
                    // std::get<2>(d)  = arma::vec4({r,0,b,1}); // DEBUG LINE
                    // debug.push_back(d); // DEBUG LINE
                }
            }

            // Put our set into the object
            classifiedImage.ballPoints.insert(classifiedImage.ballPoints.begin(), pSet.begin(), pSet.end());
            // emit(drawVisionLines(debug)); // DEBUG LINE
        }

    }  // vision
}  // modules
