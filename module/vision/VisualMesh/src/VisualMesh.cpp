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
#include "message/input/CameraParameters.h"
#include "message/input/Sensors.h"
#include "message/vision/MeshObjectRequest.h"
#include "extension/Configuration.h"
#include "message/motion/ServoTarget.h"

#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/vision.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/nubugger/NUhelpers.h"

//#include "utility/motion/RobotModels.h"

namespace module {
namespace vision {

    using message::input::Image;
    using message::input::CameraParameters;
    using message::input::Sensors;
    using message::vision::MeshObjectRequest;
    using extension::Configuration;

    //using utility::motion::kinematics::DarwinModel;
    using message::motion::ServoTarget;

    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;
    using utility::nubugger::graph;
    using utility::nubugger::drawVisionLines;


    arma::fvec3 VisualMesh::pixelToSpherical(float lambda, arma::fvec2 point){
        float px = point[0];
        float py = point[1];

        float r = std::sqrt(std::pow(px,2)+std::pow(py,2));

        float sx = std::sin(lambda * r) * (px/r);
        float sy = std::sin(lambda * r) * (py/r);
        float sz = -(std::cos(lambda*r));

        return arma::fvec3({sx, sy, sz});
    }

    arma::fvec2 VisualMesh::sphericalToPixel(float lambda, arma::fvec3 point){
        
        float r = 1.0/lambda * std::acos(-point[2]);
        if(r == 0){
            NUClear::log("cant devide by 0!");
            //break
        }

        float px = (point[0]*r) / std::sin(lambda*r);
        float py = (point[1]*r) / std::sin(lambda*r);

        return arma::fvec2({px, py});
    }

    arma::fvec3 VisualMesh::thetaPhiToCartesian(float theta, float phi){
        float r = 1.0; 

        float x = r * std::sin(theta) * std::cos(phi);
        float y = r * std::sin(theta) * std::sin(phi);
        float z = r * std::cos(theta);

        return arma::fvec3({x, y, z});
    }


    std::vector<std::pair<float, float>> thetaLimits(float phi, arma::fvec3 camera, float fovX) {
        arma::fmat::fixed<8,3> possibleVectors; //there are 8 resulting vectors
        
        float lambda = std::sin((M_PI-fovX)/2);
        NUClear::log("Lambda: ",lambda);
        float Z = 1/(std::sqrt(pow(std::cos(phi),2)+1)); //plus or minus
        NUClear::log("Z = ",Z);
        for (int i = 0; i<8; i++){//build out z component of the vectors
            possibleVectors(i,2)=( i < 4 ? Z : -Z);
        }
        
        float B = 1-pow(Z,2);
        NUClear::log("B = ", B);
        arma::fvec2 A = {(lambda - (camera[2]*Z))/camera[0], (lambda - (camera[2]*(-Z)))/camera[0]};
        NUClear::log("A = ", A);
        
        for (int i = 0; i<=1; i++){ //determine the y components
            float Y = (std::sqrt(4*(B-(A[i]*A[i]))))/2;
            NUClear::log("Y = ", Y);
            for (int j = 0; j < 4; j++){
                possibleVectors(j+i*4,1) = j < 2 ? Y : -Y;
            }
        }
        for (int i = 0; i<8; i++){ //determine the x components
            possibleVectors(i,0) = i%2==0 ? std::sqrt(B-pow(possibleVectors(i,1),2)) :  -(std::sqrt(B-pow(possibleVectors(i,1),2)));
        } 

        for (int i = 0; i<8; i++){ //print the fmat
            std::cout << possibleVectors(i,0) << " " << possibleVectors(i,1) << " " << possibleVectors(i,2) <<std::endl;
        }
        //vector must be infront of the plane
        //phi must be almost equal

        //dot of camera and camera vector needs to be greater than sin((pi-fov)/2) -- lambda
        //std::cout << "camera phi: " <<phi << std::endl;
        //NUClear::log("camera vector: ", camera);
        float cosFOV = std::cos(fovX/2);
        std::list<float> thetas;
        for (int i = 0; i < 8; i++){
            arma::fvec3 v = {possibleVectors(i,0), possibleVectors(i,1), possibleVectors(i,2)};
            float vectorPhi = std::acos(-v[2]);
            float delta = (std::numeric_limits<float>::epsilon() * 10)*phi;
            float dot = arma::dot(camera, v);

            //NUClear::log("current vector: ");
            //NUClear::log(v);
            std::cout<< " vector " << i <<": vector phi: " << vectorPhi<< " phi: " << phi << " dot: " << dot << std::endl;
            std::cout <<(std::fabs(vectorPhi-phi)<delta) << (dot<cosFOV) << std::endl;

            if(std::fabs(vectorPhi-phi)<delta && dot<cosFOV){
            //if(dot<cosFOV){
                thetas.push_back(std::atan(v[1]/v[0]));
                //NUClear::log("THETA!");
            }
        }
        
        if(thetas.size() < 2 ){
            //NUClear::log("not enough thetas!");
            //break????
        }
        std::vector<std::pair<float, float>> output;
        
        //std::cout << "thetas left: " << thetas.size() << std::endl;
        //std::cout << thetas.front() << " ----- " << thetas.back() << std::endl;
        
        output.push_back(std::make_pair(thetas.front(), thetas.back()));
         
        return output;
    }

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , lut(0.1, 0.5) { // TODO make this based of the kinematics

        on<Configuration>("VisualMesh.yaml").then([this] (const Configuration& /*config*/) {
            // Use configuration here from file VisualMesh.yaml
        });

        auto sphere = std::make_unique<MeshObjectRequest>();
        sphere->type = MeshObjectRequest::Type::SPHERE;
        sphere->radius = 0.05;
        sphere->height = 0;
        sphere->intersections = 2;
        sphere->maxDistance = 10;
        sphere->hardLimit = false;

        auto cylinder = std::make_unique<MeshObjectRequest>();
        cylinder->type = MeshObjectRequest::Type::CYLINDER;
        cylinder->radius = 0.05;
        cylinder->height = 1.15;
        cylinder->intersections = 2;
        cylinder->maxDistance = 10;
        cylinder->hardLimit = false;

        auto circle = std::make_unique<MeshObjectRequest>();
        circle->type = MeshObjectRequest::Type::CIRCLE;
        circle->radius = 0.025;
        circle->height = 0;
        circle->intersections = 1;
        circle->maxDistance = 1;
        circle->hardLimit = true;

        emit<Scope::INITIALIZE>(sphere);
        emit<Scope::INITIALIZE>(cylinder);
        emit<Scope::INITIALIZE>(circle);

        on<Trigger<MeshObjectRequest>>().then([this] (const MeshObjectRequest& request) {
            lut.addShape(request);
        });

        on<Trigger<CameraParameters>>().then([this] (const CameraParameters& params) {

            // Work out the minimum angle we can jump
            double minimumAngleJump = 2 * std::atan(M_SQRT2 / params.focalLengthPixels);

            lut.setMinimumJump(minimumAngleJump);
        });

        on<Trigger<Image>, With<Sensors>, With<CameraParameters>, Single>().then([this] (const Image&, const Sensors& sensors, const CameraParameters& /*params*/) {
            // get field of view 
            float fovX = 2.65290;//params.FOV[0];
            float fovY = 2.65290;//params.FOV[1];
            //float focalLengthPixels = params.focalLengthPixels;
            float cx = 0.0;//offset amounts
            float cy = 0.0;
            float lambda = M_PI/1000;

            // Camera height is z component of the transformation matrix
            float cameraHeight = sensors.orientationCamToGround(2, 3);

            Transform3D camMatrix = Transform3D(convert<double, 4, 4>(sensors.orientationCamToGround));

            Rotation3D camToGround = Rotation3D::createRotationZ(-Rotation3D(camMatrix.rotation()).yaw()) * camMatrix.rotation();

            arma::vec3 rotatedOriginVector = camToGround.x(); //mulitply the LOS by the translation matrix

            arma::fvec3 rotatedOrigin = arma::conv_to<arma::fvec>::from(rotatedOriginVector);



            float x = rotatedOrigin[0];
            float y = rotatedOrigin[1];
            float z = rotatedOrigin[2];

            float centerPhi = std::acos(z)/sqrt(x*x + y*y + z*z);
            //float centerTheta = std::atan(y/x);
            float minPhi = centerPhi - (fovY/2);
            float maxPhi = centerPhi + (fovY/2);
            std::cout << "minPhi: " << minPhi << " maxPhi: " << maxPhi << std::endl;

            /******************************
             * Get our edges from our LUT *
             ******************************/
            std::vector<std::pair<double, double>> lineProjections = lut.lookup(cameraHeight, minPhi, maxPhi
                , std::bind(thetaLimits, std::placeholders::_1, std::cref(rotatedOrigin), std::cref(fovX))); 

             /*************************************************************
              * Get our lookup table and loop through our phi/theta pairs *
              *************************************************************/
             std::vector<arma::fvec2> camPoints;
             arma::fvec2 imageCenter = sphericalToPixel(lambda, rotatedOrigin);
             //do i want to be applying offsets to center point???????????????????
             //imageCenter = ({imageCenter[0]-cx, imageCenter[1]-cy});

             for(auto& projection : lineProjections) {

                const float phi = projection.first;
                const float theta = projection.second;

                

                arma::fvec3 cartesianPoint = thetaPhiToCartesian(theta, phi);
                arma::fvec2 imagePoint = sphericalToPixel(lambda, cartesianPoint);
                arma::fvec2 offsetPoint = arma::fvec2({(imagePoint[0]-imageCenter[0]-cx), (imagePoint[1] - imageCenter[1] - cy)});


                camPoints.push_back(offsetPoint);

             }

             /***********************************************
              * Project our phi/theta pairs onto the screen *
              ***********************************************/
             std::vector<arma::ivec2> screenPoints;
             std::vector<std::pair<arma::ivec2, arma::ivec2>> helperPoints;


             screenPoints.reserve(camPoints.size());
             //NUClear::log(camPoints.size());
             for(auto& point : camPoints) {

                //not sure what this line does.. dont think i need it
                //arma::fvec2 floatPixel = screenCentreOffset - arma::fvec2({(focalLengthPixels * point[1] / point[0]), (focalLengthPixels * point[2] / point[0])});

                screenPoints.push_back(arma::ivec2({lround(point[0]), lround(point[1])}));
                NUClear::log(lround(point[0]),lround(point[1]));

                helperPoints.push_back(std::make_pair(screenPoints.back(), screenPoints.back() + arma::ivec2({1,1}))); 
            }
             emit(drawVisionLines(std::move(helperPoints)));
        });
    }
}
}
