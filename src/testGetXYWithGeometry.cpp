#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "CSVRow.h"
#include <sstream>
#include "getXYWithGeometry.h"

template <typename T>
std::string to_string_with_precision(const T a_value, const int n=8)
{
	std::ostringstream out;
	out << std::setprecision(n) << a_value;
	return out.str();
}

std::vector<cv::Point2d> getPointsSetDiff(const std::vector<cv::Point2d> pointsPre, const std::vector<cv::Point2d> pointsGt)
{
	std::vector<cv::Point2d> pointsSetDiff;
	for(int i=0;i<pointsPre.size();i++)
        {
        	cv::Point2d pointDiff(pointsPre[i].x-pointsGt[i].x, pointsPre[i].y-pointsGt[i].y);
        	pointsSetDiff.push_back(pointDiff);
        }
        return pointsSetDiff;
}

bool getMeanStd(double& mean, double& stdev, const std::vector<double> resultSet)
{
	if(resultSet.empty())
	{
		return false;
	}
	double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
	mean =  sum / resultSet.size(); //均值
	double accum  = 0.0;
	std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d){
		accum  += (d-mean)*(d-mean);
	});
	stdev = sqrt(accum/(resultSet.size()-1)); //标准差
	return true;
}

bool getMeanStd(double& meanX, double& stdevX, double& meanY, double& stdevY, const std::vector<cv::Point2d> pointsSet)
{
	std::vector<double> pointsX, pointsY;
	for(int i=0;i<pointsSet.size();i++)
	{
		pointsX.push_back(pointsSet[i].x);
		pointsY.push_back(pointsSet[i].y);
		
	}
	getMeanStd(meanX, stdevX, pointsX);
	getMeanStd(meanY, stdevY, pointsY);
	return true;
}

void getMap1Map2(cv::Mat& P, cv::Mat& map1, cv::Mat& map2, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Size imageSize)
{
	double alpha=0;//0 kuo, 1 suo
	cv::Size rectificationSize = imageSize;
	cv::Rect validPixROI;
	P= getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,
					imageSize,
					alpha,//0 huo, 1 suo
					rectificationSize,//undistorted image size
					&validPixROI//undistorted image rectangle in source image
					);//new camera matirx

	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
				P,//undistort image camera instrinsic matirx
				rectificationSize,//undistorted image size
				CV_16SC2, map1, map2);

}


std::vector<std::string> splitString(const std::string& s, const std::string& c)
{
	std::vector<std::string> v;
	std::string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while(std::string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2-pos1));

	pos1 = pos2 + c.size();
	pos2 = s.find(c, pos1);
	}
	if(pos1 != s.length())
		v.push_back(s.substr(pos1));
	return v;
}

std::vector<cv::Point> getPoints(std::string fileName)
{
	std::ifstream myfile(fileName);
	std::string temp;
	std::vector<cv::Point> ret;
	while(getline(myfile,temp)) //按行读取字符串 
	{
		std::vector<std::string> pointsStr = splitString(temp, ",");
		ret.push_back(cv::Point(std::atoi(pointsStr[0].c_str()), std::atoi(pointsStr[1].c_str())));
	} 
	myfile.close();
	return ret;
}

std::vector<cv::Point2d> getPoints2D(std::string fileName)
{
	std::ifstream myfile(fileName);
	std::string temp;
	std::vector<cv::Point2d> ret;
	while(getline(myfile,temp)) //按行读取字符串 
	{
		std::vector<std::string> pointsStr = splitString(temp, ",");
		ret.push_back(cv::Point2d(std::atof(pointsStr[0].c_str()), std::atof(pointsStr[1].c_str())));
	} 
	myfile.close();
	return ret;
}

std::vector<cv::Point2d> transformPointsIntToDouble(const std::vector<cv::Point> ctrlPoints)
{
	std::vector<cv::Point2d> retPoint2d;
	for(int i=0;i<ctrlPoints.size();i++)
	{
		retPoint2d.push_back(cv::Point2d(double(ctrlPoints[i].x),double(ctrlPoints[i].y)));
	}
	return retPoint2d;
}

void getParserInfo(std::string& intricFile, std::string& inputImg, std::string& ctrlPixFile, std::string& ctrlDxDyFile, std::string& outputImg, int argc, char** argv)
{
	cv::CommandLineParser parser(argc, argv, "{help||}{intric|../data/cam.yml|camera intrinsic matrix}"
                             "{i|../data/2020-07-25-12-38-34.jpg|input image}"
                             "{iPix|../data/ctrlpix.txt|input control point}"
                             "{iDxDy|../data/ctrldxdy.txt|input control point real dx, dy}"
                             "{o|../data/ImageEstimateright0.75BirdView.png| birdview image}");

	intricFile = parser.get<std::string>("intric");
	inputImg = parser.get<std::string>("i");
	ctrlPixFile = parser.get<std::string>("iPix");
	ctrlDxDyFile = parser.get<std::string>("iDxDy");
	outputImg = parser.get<std::string>("o");
} 

bool getCameraMatrixAndDistortionCoefficients(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::string intricFile)
{
	cv::FileStorage fs(intricFile, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		std::cout<<"Failed to open file "<<intricFile<<std::endl;
		return false;
	}
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	fs.release();
	return true;
}



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
	if(argc<2)
	{
		help();
		return 0;
	}
	
	std::string intricFile, inputImg, ctrlPixFile, ctrlDxDyFile, outputImg;
	getParserInfo(intricFile, inputImg, ctrlPixFile, ctrlDxDyFile, outputImg, argc, argv);
	std::cout<< "intric:"<<intricFile<<std::endl<< "inputImg:"<<inputImg<<std::endl<<"ctrlPixFile:"<<ctrlPixFile<<std::endl
	<<"ctrlDxDyFile:"<<ctrlDxDyFile<<std::endl<<"outputImg:"<<outputImg<<std::endl;

	cv::Mat cameraMatrix, distCoeffs;
	bool ret = getCameraMatrixAndDistortionCoefficients(cameraMatrix, distCoeffs, intricFile);
	std::cout<<"camera_matrix:"<<std::endl<<cameraMatrix<<std::endl;
	std::cout<<"distortion_coefficients:"<<std::endl<<distCoeffs<<std::endl;

	cv::Mat img = cv::imread(inputImg, 1);
	if(img.empty())
	{
		std::cout<<"The image is empty"<<std::endl;
		return -1;
	}
        
        cv::Mat P, map1, map2;
        getMap1Map2(P, map1, map2, cameraMatrix, distCoeffs, img.size());
	std::cout << "P: "<<P<<std::endl;
        
        std::vector<cv::Point> ctrlPoints = getPoints(ctrlPixFile);
        for(int i=0;i<ctrlPoints.size();i++)
        {
        	cv::circle(img, ctrlPoints[i], 3, cv::Scalar(255, 0, 0), -1);
        }
        
	std::vector<cv::Point2d> ctrlRealDxDy = getPoints2D(ctrlDxDyFile);
        
        cv::Mat undistortImg;
        cv::remap(img, undistortImg, map1, map2, cv::INTER_LINEAR);
        
        std::vector<cv::Point2d> undistortCtrlPoints;
        cv::undistortPoints(transformPointsIntToDouble(ctrlPoints), undistortCtrlPoints, cameraMatrix, distCoeffs, cv::Mat(), P);
        for(int i=0;i<undistortCtrlPoints.size();i++)
        {
//        	std::cout<<undistortCtrlPoints[i]<<std::endl;
        	cv::circle(undistortImg, undistortCtrlPoints[i], 8, cv::Scalar(0, 0, 255), 1);
        }
        

        

        withGeometry::TiLane laneDxDy;
        std::vector<cv::Point2d>lines;
//	lines.push_back(cv::Point2d(731.712, 916.204));//leftbottom
//	lines.push_back(cv::Point2d(993.235, 618.421));//lefttop
//	lines.push_back(cv::Point2d(1423.62, 924.942));//rightbottom
//	lines.push_back(cv::Point2d(1104.93, 618.452));//righttop
	lines.push_back(cv::Point2d(1380, 1665));//leftbottom
	lines.push_back(cv::Point2d(1479, 855));//lefttop
	lines.push_back(cv::Point2d(2155, 1686));//rightbottom
	lines.push_back(cv::Point2d(1877, 861));//righttop
        cv::Point2d crossPoint = laneDxDy.getCrossPoint(lines);
        double cameraPitch = laneDxDy.getPitch(P, crossPoint);
        std::cout<<"pitch: "<<cameraPitch<<std::endl;
        std::cout<<"crossPoint: "<<crossPoint<<std::endl;
        cv::circle(undistortImg, crossPoint, 4, cv::Scalar(0, 255, 0), -1);
        
        double h = 1.761;
//        std::vector<cv::Point2d> lanePointsDxDy = laneDxDy.getPointsXY(undistortCtrlPoints, P, h);
        std::vector<cv::Point2d> lanePointsDxDy = laneDxDy.getPointSetsXY(undistortCtrlPoints, crossPoint, P, h, cameraPitch);
        std::vector<cv::Point2d> pointsSetDiff = getPointsSetDiff(lanePointsDxDy, ctrlRealDxDy);
        std::cout<<"prediction, groundtruth, difference"<<std::endl;
        for(int i=0;i<lanePointsDxDy.size();i++)
        {
//        	cv::Point2d pointDiff(lanePointsDxDy[i].x-ctrlRealDxDy[i].x, lanePointsDxDy[i].y-ctrlRealDxDy[i].y);
        	std::cout<<lanePointsDxDy[i]<<" , "<<ctrlRealDxDy[i]<<" , "<<pointsSetDiff[i]<<std::endl;
        }
        
        double meanX, stdevX,  meanY, stdevY;
        getMeanStd(meanX, stdevX, meanY, stdevY, pointsSetDiff);
        std::cout<<"X error mean, std: "<<meanX<<" , "<<stdevX<<std::endl<<"Y error mean, std: "<<meanY<<" , "<<stdevY<<std::endl;
        
	double scale=0.5;
        cv::Mat imgShow, undistortImgShow;
        cv::resize(img, imgShow, cv::Size(0,0), scale,scale, cv::INTER_LINEAR); 
        cv::resize(undistortImg, undistortImgShow, cv::Size(0,0), scale, scale, cv::INTER_LINEAR);
        cv::namedWindow("rawImg",0);
        cv::imshow("rawImg", imgShow);
        cv::namedWindow("undistortImg",0);
        cv::imshow("undistortImg", undistortImgShow);
        cv::waitKey();
        
	return 0;
}
