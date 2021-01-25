#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include "getXYWithHomograph.h"
#include "CSVRow.h"
using namespace cv;
using namespace std;

template <typename T>
std::string to_string_with_precision(const T a_value, const int n=8)
{
	std::ostringstream out;
	out << std::setprecision(n) << a_value;
	return out.str();
}

std::istream& operator>>(std::istream& str, CSVRow& data)
{
	data.readNextRow(str);
	return str;
}



const char * usage = 
"\n"
"./testGetXYWithHomograph ../data/blueCar_H_15m.xml ../data/data.csv ../data/output.csv"
"\n";

static void help()
{
	cout << usage;
}
int main(int argc, char** argv)
{
	if(argc<2)
	{
		help();
		return 0;
	}

	std::string H_file = argv[1];
	std::string inputFileName = argv[2];
	std::string outputFileName = argv[3];

	cv::FileStorage fs;
	cv::Mat H(3,3,CV_64F);
	if(fs.open(H_file,cv::FileStorage::READ))
	{
		fs["H"] >> H;
		std::cout << "H matrix -------------------------------"<<endl;
		std::cout << H <<endl;
	}else
	{
		std::cout << "Open H file error!!!"<<endl;
	}

	std::ifstream inputFile(inputFileName);
	std::ofstream outputFile(outputFileName);
	CSVRow row;
	inputFile >> row;
	std::cout << "--------------------------------" <<endl;
	std::cout << "Input file name " << inputFileName <<endl;
	std::string title = "realx, realy";
	outputFile << title <<endl;
	std::cout << "image point, realdxdy---------------------"<<endl;
	while(inputFile >> row)
	{
		cv::Point2f point;
		point.x = int(std::stod(row[0]));
		point.y = int(std::stod(row[1]));

		cv::Point2d pos = withHomograph::getDyDx(H, point);
		//getPosition(dy,dx, point);
		std::string writeRow = to_string_with_precision(pos.x) + "," + to_string_with_precision(pos.y);
		outputFile << writeRow <<endl;
		std::cout << point<< " , [" << pos.x << " , " << pos.y << "]"<<endl;
		
	}
	inputFile.close();
	outputFile.close();

	return 0;
}
