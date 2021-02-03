#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include "getCarInWitchLane.h"


const char * usage = 
"\n"
"./testGetCarInWitchLane"
"\n";

static void help()
{
	std::cout << usage;
}
int main(int argc, char** argv)
{
	std::string maskFileName = "../data/beiMask.png";
	cv::Mat maskLane = cv::imread(maskFileName, cv::IMREAD_UNCHANGED);
	
	int x = 2005;//1991,394,29,26
	int y = 420;
	
	std::cout<<"test Point : "<< x<<" , "<< y<<std::endl;
	CARPOSINFO ret = getLaneIndAndOffSet(x, y, maskLane);
	std::cout<<ret.laneInd<<" "<<ret.offSetXRefLeftLane<<std::endl;
	cv::circle(maskLane, cv::Point(x,y), 8, cv::Scalar(0,0,0),-1);
	cv::resize(maskLane, maskLane, cv::Size(maskLane.cols/3, maskLane.rows/3));
	cv::imshow("testShow", maskLane);
	cv::waitKey(0);
	return 0;
}
