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

std::vector<cv::Point2d> getPointsSetDiff(const std::vector<cv::Point2d> pointsPre, const std::vector<cv::Point3d> pointsGt)
{
	std::vector<cv::Point2d> pointsGt2d;
	for(int i=0;i<pointsGt.size();i++)
	{
		pointsGt2d.push_back(cv::Point2d(pointsGt[i].x,pointsGt[i].y));
	}
	return getPointsSetDiff(pointsPre, pointsGt2d);
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

void getRealCsv(std::vector<cv::Point3d>& carPos, std::vector<cv::Point>& sign, std::vector<cv::Point3d>& signRTKGt, std::vector<cv::Point3d>& signPnPGt, const std::string fileName)
{
	std::ifstream myfile(fileName);
	std::string temp;
	getline(myfile,temp);
	getline(myfile,temp);
	while(getline(myfile,temp)) //按行读取字符串 
	{
		std::vector<std::string> pointsStr = splitString(temp, ",");
		carPos.push_back(cv::Point3d(std::atof(pointsStr[1].c_str()), std::atof(pointsStr[2].c_str()), std::atof(pointsStr[3].c_str())));
		sign.push_back(cv::Point(std::atoi(pointsStr[7].c_str()), std::atoi(pointsStr[8].c_str())));
		sign.push_back(cv::Point(std::atoi(pointsStr[9].c_str()), std::atoi(pointsStr[10].c_str())));
		sign.push_back(cv::Point(std::atoi(pointsStr[11].c_str()), std::atoi(pointsStr[12].c_str())));
		sign.push_back(cv::Point(std::atoi(pointsStr[13].c_str()), std::atoi(pointsStr[14].c_str())));
		signRTKGt.push_back(cv::Point3d(std::atof(pointsStr[15].c_str()), std::atof(pointsStr[16].c_str()), std::atof(pointsStr[17].c_str())));
		signRTKGt.push_back(cv::Point3d(std::atof(pointsStr[18].c_str()), std::atof(pointsStr[19].c_str()), std::atof(pointsStr[20].c_str())));
		signRTKGt.push_back(cv::Point3d(std::atof(pointsStr[21].c_str()), std::atof(pointsStr[22].c_str()), std::atof(pointsStr[23].c_str())));
		signRTKGt.push_back(cv::Point3d(std::atof(pointsStr[24].c_str()), std::atof(pointsStr[25].c_str()), std::atof(pointsStr[26].c_str())));
		signPnPGt.push_back(cv::Point3d(std::atof(pointsStr[27].c_str()), std::atof(pointsStr[28].c_str()), std::atof(pointsStr[29].c_str())));
		signPnPGt.push_back(cv::Point3d(std::atof(pointsStr[30].c_str()), std::atof(pointsStr[31].c_str()), std::atof(pointsStr[32].c_str())));
		signPnPGt.push_back(cv::Point3d(std::atof(pointsStr[33].c_str()), std::atof(pointsStr[34].c_str()), std::atof(pointsStr[35].c_str())));
		signPnPGt.push_back(cv::Point3d(std::atof(pointsStr[36].c_str()), std::atof(pointsStr[37].c_str()), std::atof(pointsStr[38].c_str())));
	} 
	myfile.close();
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

void getParserInfo(std::string& intricFile, std::string& csvFile, std::string& stepImage, std::string& curImage, int argc, char** argv)
{
	cv::CommandLineParser parser(argc, argv, "{help||}{intric|../data/duantoulu/duantouImgUndistortkuo.xml|camera intrinsic matrix}"
                             "{i|../data/duantoulu/real.csv|csv file}"
                             "{i0|../data/duantoulu/550.jpg|step image}"
                             "{i1|../data/duantoulu/650.jpg|current image}");

	intricFile = parser.get<std::string>("intric");
	csvFile = parser.get<std::string>("i");
	stepImage = parser.get<std::string>("i0");
	curImage = parser.get<std::string>("i1");
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




double getStepDis(const cv::Point3d stepCarPos, const cv::Point3d curCarPos)
{
	return sqrt((stepCarPos.x-curCarPos.x)*(stepCarPos.x-curCarPos.x)+(stepCarPos.y-curCarPos.y)*(stepCarPos.y-curCarPos.y));
}

int carPosIndex[11] = {4,9,10,15,19,23,27,31,35,39,43};
std::vector<double> getStepDisSets(const std::vector<cv::Point3d> carPos, const int startFrameIndex , const int endFrameIndex)
{
	std::vector<double> retStepDis;
	for(int i=startFrameIndex;i<endFrameIndex;i++)
	{
		retStepDis.push_back(getStepDis(carPos[carPosIndex[i]], carPos[carPosIndex[i+1]]));
	}
	return retStepDis;
}

template<typename T>
std::vector<T> RangeCopy(std::vector<T> v, int startIndex, int count)
{
	return std::vector<T>(v.begin()+startIndex*4, v.begin()+startIndex*4+count);
}

template<typename T>
std::vector<T> getSignPointsFrom(const std::vector<T> sign, const int FrameIndex, const int signIndex=0)
{
	return RangeCopy(sign, carPosIndex[FrameIndex], 4);
}

template<typename T>
std::vector<T> getSignCorner(const std::vector<T> sign, const int startFrameIndex , const int endFrameIndex)
{
	std::vector<T> retSignCorner;
	for(int i=startFrameIndex; i<=endFrameIndex; i++)
	{
		retSignCorner.push_back(sign[carPosIndex[i]]);
	}
	return retSignCorner;
}

std::vector<withGeometry::SignCorner> transforPointSeq2SignCorner(const std::vector<cv::Point> sign)
{
	std::vector<withGeometry::SignCorner> retSignCorner;
	for(int i=0;i<sign.size();i+=4)
	{
		withGeometry::SignCorner signCorner;
		signCorner.corner.push_back(cv::Point2d(sign[i+0]));
		signCorner.corner.push_back(cv::Point2d(sign[i+1]));
		signCorner.corner.push_back(cv::Point2d(sign[i+2]));
		signCorner.corner.push_back(cv::Point2d(sign[i+3]));
		retSignCorner.push_back(signCorner);
	}
	return retSignCorner;
	
}

std::vector<cv::Point2d> getSignCornerPoints(const withGeometry::SignCorner signCorner)
{
	return signCorner.corner;
}

const char * usage = 
"\n"
"./testGetXYSignWithGeometry -intric=../data/duantoulu/duantouImgUndistortkuo.xml -i=../data/duantoulu/real.csv -i0=../data/duantoulu/550.jpg -i1=../data/duantoulu/650.jpg"
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
	
	std::string intricFile, csvFile, stepImage, curImage;
	getParserInfo(intricFile, csvFile, stepImage, curImage, argc, argv);
	std::cout<< "intric:"<<intricFile<<std::endl<< "csvFile:"<<csvFile<<std::endl<<"stepImage:"<<stepImage<<std::endl
	<<"curImage:"<<curImage<<std::endl;

	cv::Mat cameraMatrix, distCoeffs;
	bool ret = getCameraMatrixAndDistortionCoefficients(cameraMatrix, distCoeffs, intricFile);
	std::cout<<"camera_matrix:"<<std::endl<<cameraMatrix<<std::endl;
	std::cout<<"distortion_coefficients:"<<std::endl<<distCoeffs<<std::endl;

	cv::Mat img = cv::imread(curImage, 1);
	if(img.empty())
	{
		std::cout<<"The image is empty"<<std::endl;
		return -1;
	}
        
//        cv::Mat P, map1, map2;
//        getMap1Map2(P, map1, map2, cameraMatrix, distCoeffs, img.size());
//	std::cout << "P: "<<P<<std::endl
        
        std::vector<cv::Point3d> carPos;
        std::vector<cv::Point> sign;
        std::vector<cv::Point3d> signRTKGt;
        std::vector<cv::Point3d> signPnPGt;
        getRealCsv(carPos, sign, signRTKGt, signPnPGt, csvFile);
        cv::Mat undistortImg = img;
        
        
        int startIndex = 3;
        int curIndex = 7;
        
//        double stepDis = stepDisSets[0];
////        double stepDis = getStepDis(carPos[stepIndex], carPos[curIndex]);
//        std::cout<<"step: "<<stepDis<<std::endl;

        withGeometry::TiLane laneDxDy;
        withGeometry::TiSign signDxDy;
        std::vector<cv::Point2d>lines;
	lines.push_back(cv::Point2d(337, 657));
	lines.push_back(cv::Point2d(576, 435));
	lines.push_back(cv::Point2d(983, 662));
	lines.push_back(cv::Point2d(735, 450));
	for(int i=0;i<lines.size();i++)
	{
		cv::circle(undistortImg, lines[i], 4, cv::Scalar(0, 255, 0), -1);
	}
        cv::Point2d crossPoint = laneDxDy.getCrossPoint(lines);
        double cameraPitch = laneDxDy.getPitch(cameraMatrix, crossPoint);
        std::cout<<"pitch: "<<cameraPitch<<std::endl;
        cv::circle(undistortImg, crossPoint, 4, cv::Scalar(0, 255, 0), -1);
        
        std::vector<cv::Point> showSign = getSignPointsFrom(sign, curIndex);
        for(int i=0;i<4;i++)
        {
        	cv::circle(undistortImg, showSign[i], 4, cv::Scalar(0, 255, 0), -1);	
        }
        
        double h = 1.761;
        std::vector<double> stepDisSets = getStepDisSets(carPos, startIndex , curIndex);
        std::vector<withGeometry::SignCorner> signCorner = getSignCorner(transforPointSeq2SignCorner(sign), startIndex , curIndex);
        std::vector<cv::Point2d> signPointsDxDy = getSignCornerPoints(signDxDy.getPointXY(stepDisSets, signCorner,  crossPoint, cameraMatrix));
        
        std::vector<cv::Point2d> pointsSetDiff = getPointsSetDiff(signPointsDxDy, getSignPointsFrom(signRTKGt, curIndex));
        std::cout<<"prediction, groundtruth, difference"<<std::endl;
        for(int i=0;i<signPointsDxDy.size();i++)
        {
//        	cv::Point2d pointDiff(lanePointsDxDy[i].x-ctrlRealDxDy[i].x, lanePointsDxDy[i].y-ctrlRealDxDy[i].y);
        	std::cout<<signPointsDxDy[i]<<" , "<<getSignPointsFrom(signRTKGt,curIndex)[i]<<" , "<<pointsSetDiff[i]<<std::endl;
        }
        
        double meanX, stdevX,  meanY, stdevY;
        getMeanStd(meanX, stdevX, meanY, stdevY, pointsSetDiff);
        std::cout<<"X error mean, std: "<<meanX<<" , "<<stdevX<<std::endl<<"Y error mean, std: "<<meanY<<" , "<<stdevY<<std::endl;
        
	double scale=0.5;
        cv::Mat undistortImgShow;
        cv::resize(undistortImg, undistortImgShow, cv::Size(0,0), scale, scale, cv::INTER_LINEAR);
        cv::imshow("undistortImg", undistortImgShow);
        cv::waitKey();
        
	return 0;
}
