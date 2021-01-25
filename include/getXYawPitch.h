#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
using namespace cv;
using namespace std;

void getCrossPoint(std::vector<cv::Point2d>lines,std::vector<cv::Point2d>& cross_Pt);
void transformToBirdViewLine(std::vector<cv::Point2d>& birdViewLines, std::vector<cv::Point2d> lines, cv::Mat m);
/**************************************************************************************************************************
 * Function: one:get the offset along the x direction, which means the dirction that is vertical to the lane.(Unit:m)
 *           two:get the yaw reference to the lane.(Unit:degree)
 *           three:get the pitch of the camera.(Unit:degree)
 * Parameters:
 *          offx: output, offset along the x direction.
 *          yaw: output, yaw reference to the lane.
 *          pitch: output, pitch of the camera.
 *          imageSize: input, the image size,default is (720,1280).
 *          lines: input, only two lane is supported. The line order in the lines looks like below
 *             1/  \3
 *             /    \
 *           0/      \2
 *          vanishPoint: input, vanishing point in the image. The default is (0,0), which means unknown.
 *          intrinsicMatrix: input, intrinsic matrix of the camera.
 *          distCoeffs: input, distorting coefficients.
 *          H: input, homography matirx
 *          condis: input, the distance of the position the image bottom and the camera,default is zero, which means has no idea about it.
 * return:True if everything is OK, or false.
 *
 *************************************************************************************************************************/
bool getXYawPitch(double& offx, double& yaw, double& pitch, std::vector<cv::Point2d> lines, cv::Mat intrinsicMatrix, cv::Mat distCoeffs, cv::Mat H, Size imageSize=Size(720,1280),cv::Point2d vanishPoint=cv::Point2d(0,0), double condis=0);
