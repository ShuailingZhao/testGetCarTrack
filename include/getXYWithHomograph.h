#ifndef TILANE_H
#define TILANE_H

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
namespace withHomograph
{
	cv::Point2f transferPoint(cv::Point2f point);
	cv::Point2f getDisPoint(cv::Rect rect);
	cv::Mat getDis(cv::Mat m, cv::Mat points);
	void getRealDyDx(cv::Point2f p, double& dy,double& dx);
	cv::Point2d getDyDx(cv::Mat m, cv::Point2f point);


}
#endif


