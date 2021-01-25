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
"./testGetXYWithGeometry -intric=../data/cam.yml -i=../data/2020-07-25-12-38-34.jpg -o=../data/ImageEstimateright0.75BirdView.png"
"\n";

static void help()
{
	std::cout << usage;
}
int main(int argc, char** argv)
{
	std::string maskFileName = "../data/nanMask.png";
	cv::Mat maskLane = cv::imread(maskFileName, cv::IMREAD_UNCHANGED);
	
	int x = 1725;
	int y = 1152;
	
	std::cout<<"test Point : "<< x<<" , "<< y<<std::endl;
	CARPOSINFO ret = getLaneIndAndOffSet(x, y, maskLane);
	std::cout<<ret.laneInd<<" "<<ret.offSetXRefLeftLane<<std::endl;
	return 0;
}
