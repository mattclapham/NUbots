#include "Horizon.h"
#include <cmath>

Horizon::Horizon()
{
  Reset();
  return;
}

Horizon::~Horizon()
{
  return;
}

void Horizon::Reset(){
  exists = false;
  return;
}

void Horizon::Calculate(double bodyPitch, double bodyRoll, double headYaw, double headPitch, int cameraNumber)
// Calculate the two edge ponts on the screen at which the horizon line intersects. Maths based on the 2005 German Team AIBO report.
{
    double s = 160/2.0;
    double alpha = (0.7854/2.0);
    // Equations generated by multiplying the y and z components of a 3D rotation matrix together
    double cotAlpha;
    if(tan(alpha) == 0){
      cotAlpha = 1000000000;
    } else {
      cotAlpha = 1.0/tan(alpha);
    }

    if(cameraNumber == 1)
      headPitch += 0.6981;

//  2008 Values ... I think they were wrong...
//  float r31 = sin(headYaw)*sin(bodyRoll)+(cos(headYaw)*cos(headTilt)*sin(bodyTilt)+cos(headYaw)*sin(headTilt)*cos(bodyTilt))*cos(bodyRoll);
//    float r32 = -cos(headYaw)*sin(bodyRoll)+(sin(headYaw)*cos(headTilt)*sin(bodyTilt)+sin(headYaw)*sin(headTilt)*cos(bodyTilt))*cos(bodyRoll);
//    float r33 = (-sin(headTilt)*sin(bodyTilt)+cos(headTilt)*cos(bodyTilt))*cos(bodyRoll);


// 2009 Values ->  Moved the pan rotation to start, since head pan is not effected by head tilt in Nao.
    float r31 = cos(bodyRoll)*(cos(bodyPitch)*sin(headPitch) + cos(headYaw)*cos(headPitch)*sin(bodyPitch)) - cos(headPitch)*sin(bodyRoll)*sin(headYaw);
    float r32 = cos(bodyRoll)*sin(bodyPitch)*sin(headYaw) + cos(headYaw)*sin(bodyRoll);
    float r33 = cos(bodyRoll)*(cos(bodyPitch)*cos(headPitch) - cos(headYaw)*sin(bodyPitch)*sin(headPitch)) + sin(bodyRoll)*sin(headYaw)*sin(headPitch);

//    Old 210 AIBO Equations
//    float r31 = -cos(bodyRoll)*sin(headPitch)*cos(headYaw) + sin(bodyRoll)*sin(headPitch);
//    float r32 = cos(bodyRoll)*sin(headPitch)*sin(headYaw)+sin(bodyRoll)*cos(headYaw);
//    float r33 = cos(bodyRoll)*cos(headPitch);

    if(r33 == 0) r33 = 0.00001;
// Old calculation.
//    float zl = IMAGE_HEIGHT/2 + s*(r32  + r31*cotAlpha)/r33;
//    float zr = IMAGE_HEIGHT/2 + s*(-r32 + r31*cotAlpha)/r33;
    Point zl,zr;
    zl.x = 0;
    zl.y = 120/2 - s*(-r32  + r31*cotAlpha)/r33;
    zr.x = 160;
    zr.y = 120/2 - s*(r32 + r31*cotAlpha)/r33;
    setLineFromPoints(zl,zr);
//    std::cout << "Horizon: zl = " << zl.y << " zr = " << zr.y << std::endl;
    exists = true;
    return;
}

bool Horizon::IsBelowHorizon(int x, int y){
  return findYFromX(x) < y;
}
