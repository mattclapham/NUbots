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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#include "VisualMesh.h"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/support/Configuration.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace vision {
	
	using message::input::Image;
	using message::input::Sensors;
    using message::support::Configuration;

    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;

    struct MeshObjectRequest {
    	enum Type {
    		SPHERE,
    		CIRCLE,
    		CYLINDER
    	};
    };

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("VisualMesh.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file VisualMesh.yaml
        });

        on<Trigger<MeshObjectRequest>>().then([] (const MeshObjectRequest&) {
        	// Update list of shapes
        	// Regnerate LUT
        });

        on<Trigger<Image>, With<Sensors>>().then([] (const Image& image, const Sensors& sensors) {

        	// Go nuts!
        	//image.width;
        	//image.height;

        	//Find Angular Limits
        	//Phi = ?????
        	//Theta
        	// get FOV_X and FOV_Y
        	//Find Corner Points //WRITTEN
        	//Transform them into world space //USE EXISTING FROM VISION.h
        	//Find the four Points of Intersection //FUNCTION WRITTEN
        	//Determine which of the POI's fall on or within the circle //FUNCTION NOT WRITTEN
        	//Abide by cases to find theta limits //FUNCTION NOT WRITTEN

        	//For each phi, jump by theta within the limits, //NOT WRITTEN 
        	// to find the sample point and convert it to camera space //WRITTEN





        });
    }

	std::vector<arma::vec3> VisualMesh::findCornerPoints(double xmax, double FOV_X, double FOV_Y) {
		double ymax = viewingAngleMax(xmax, FOV_X);
		double zmax = viewingAngleMax(xmax, FOV_Y);

		// calculates the corner points of the camera's field of view in camera space at a given x.
		std::vector<arma::vec3> corners;
		corners.push_back(arma::vec3({xmax, ymax, zmax}));
		corners.push_back(arma::vec3({xmax, -ymax, zmax}));
		corners.push_back(arma::vec3({xmax, ymax, -zmax}));
		corners.push_back(arma::vec3({xmax, -ymax, -zmax}));
		return corners;
	}

	arma::vec3 VisualMesh::convertPhiAndThetaToCamSpace(double phiDash, double theta, arma::mat44 camToGround) {
		// phi and theta are converted to spherical coordinates in a space with the same
		// orientation as the world, but with origin at the camera position. 
		// NOTE: the associated phi in spherical coords is given by -(pi/2 - phiDash), which simplifies the spherical coord conversion to:
		arma::vec3 sphericalCoords = {std::cos(theta)*std::sin(phiDash), std::sin(theta)*std::sin(phiDash), -std::cos(phiDash)};
		// To put in camera space multiply by the rotation matrix, use 0 for rotation
		arma::vec4 camSpacePoint_ = camToGround.i()*arma::join_cols(sphericalCoords, arma::vec({0}));
		// Discard the 4th element (= 0)
		arma::vec3 camSpacePoint = camSpacePoint.submat(0,0,2,0);
		return camSpacePoint;
		// Then use arma::vec2 projectCamSpaceToScreen(const arma::vec3& point, const double& camFocalLengthPixels) in Vision.h to convert to screen space.
	}

}
}
