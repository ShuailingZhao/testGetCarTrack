#ifndef GETANGLE
#define GETANGLE
#include <iostream>
using namespace std;
///////////////////////////////////////////////////////////////////////////////////
/// \brief getAng
/// \param Width:The width of the image
/// \param Height:The height of the image
/// \param Fy:focal length (pixel unit)
/// \param Hy:pixel size
/// \param P0x:left top point x coordinate
/// \param P0y:left top point y coordinate
/// \param P1x:left bottom point x coordinate
/// \param P1y:left bottom point y coordinate
/// \param P2x:right top point x coordinate
/// \param P2y:right top point y coordinate
/// \param P3x:right bottom point x coordinate
/// \param P3y:right bottom point y coordinate
/// \param Heading:camera heading about the car
/// \param Pitch:camera pitch about the car
/// \param Roll:camera roll about the car
/// \param HOverGround:height of camera install position over the ground
/// \return Angle of the car heading about the lane
///////////////////////////////////////////////////////////////////////////////////
double getAng(const double Width, const double Height, const double Fy,const double Hy,\
              const double P0x, const double P0y, const double P1x, const double P1y,\
              const double P2x, const double P2y, const double P3x, const double P3y,\
	      const double P4x, const double P4y,\
              const double Heading, const double Pitch, const double Roll,\
              const double HOverGround);
#endif
