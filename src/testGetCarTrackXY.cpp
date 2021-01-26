#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include "CSVRow.h"
#include <sstream>
#include "getXYWithGeometry.h"
#include "wgs84_3degreeGauss.h"
#include "getCarInWitchLane.h"
#include "ToolsBox.h"

//#define ZHAODEBUG
struct CARDETECTED{
	int frame;
	int carId;
	cv::Rect bbox;
	double width;
};
template <typename T>
std::string to_string_with_precision(const T a_value, const int n=13)
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

void getParserInfo(std::string& configFile, int argc, char** argv)
{
	cv::CommandLineParser parser(argc, argv, "{help||}{config|../data/defaultConfig.yml|config file}");
	configFile = parser.get<std::string>("config");
} 

bool getConf(cv::Mat& P, cv::Point3d& cameraPos, cv::Point2d& crossPoint, double& roadHeading, double& cameraPitch, double& cameraHeigth, std::string& videoName, std::string& maskFileName, std::string& intrinsicName, std::string& hdmapName, std::string& carDataName, std::string& writeCarTrackName, std::string configFile)
{
	cv::FileStorage fs(configFile, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		std::cout<<"Failed to open file "<<configFile<<std::endl;
		return false;
	}
	cv::Mat crossPointMat;
	cv::Mat cameraPosition;
	fs["cameraVideoName"] >> videoName;
	fs["maskFileName"] >> maskFileName;
	fs["intrinsic"] >> intrinsicName;
	fs["hdMapName"] >> hdmapName;
	fs["carDataName"] >> carDataName;
	fs["writeCarTrackName"] >> writeCarTrackName;
	fs["P"] >> P;
	fs["cameraPos"] >> cameraPosition;
	fs["cameraPitch"] >> cameraPitch;
	fs["cameraHeight"] >> cameraHeigth;
	fs["crossPoint"] >> crossPointMat;
	fs["roadHeading"] >> roadHeading;
	fs.release();
	crossPoint = cv::Point2d{crossPointMat.at<double>(0,0), crossPointMat.at<double>(0,1)};
	cameraPos = cv::Point3d{cameraPosition.at<double>(0,0), cameraPosition.at<double>(0,1), cameraPosition.at<double>(0,1)};
	return true;
}

std::ifstream& operator>>(std::ifstream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}

std::vector<std::vector<cv::Point2d>> getLaneSequent(const std::string hdmapName)
{
	std::vector<std::vector<cv::Point2d>> laneSequent;
	std::ifstream laneCsvFile(hdmapName);
	CSVRow csvRow;
	int laneCount=0;
	while((laneCsvFile>>csvRow) && csvRow.size()>1)
	{
		std::vector<cv::Point2d> oneLane;
		for(int i=0;i<csvRow.size();i+=2)
		{
			oneLane.push_back(cv::Point2d(std::stod(csvRow[i]), std::stod(csvRow[i+1])));	
		}
		laneSequent.push_back(oneLane);
		laneCount++;
		
		
	}
	laneCsvFile.close();
	return laneSequent;
}

bool getCarDetect(std::vector<CARDETECTED>& carInOneFrame, std::ifstream& carPosCsvFile)
{
	if(!carInOneFrame.empty())
	{
		carInOneFrame.clear();
	}
	bool ret = false;
	CSVRow csvRow;
	if((carPosCsvFile>>csvRow) && csvRow.size()>=1)
	{
		int oneCarInfoLength=6;
		for(int i=0;i<csvRow.size()/oneCarInfoLength;i++)
		{
			int startCol = oneCarInfoLength*i+1;
			CARDETECTED detectCar;
			detectCar.frame = stoi(csvRow[0]);
			detectCar.carId = stoi(csvRow[startCol]);
			detectCar.bbox = cv::Rect{stoi(csvRow[startCol+1]), stoi(csvRow[startCol+2]), stoi(csvRow[startCol+3]), stoi(csvRow[startCol+4])};
			detectCar.width = stod(csvRow[startCol+5]);
			carInOneFrame.push_back(detectCar);	
		}	
		ret = true;
	}
//	carPosCsvFile.close();
	return ret;
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

std::vector<cv::Point2d> extractCarMidPoint(std::vector<CARDETECTED> carDetectInfo)
{
	std::vector<cv::Point2d> carMidPoints;
	for(int i=0;i<carDetectInfo.size();i++)
	{
		carMidPoints.push_back(cv::Point2d{double(carDetectInfo[i].bbox.x+carDetectInfo[i].bbox.width/2.0), double(carDetectInfo[i].bbox.y+carDetectInfo[i].bbox.height)});
	}
	return carMidPoints;
}

Eigen::Matrix<double,3,3> getRx(const double theta_x)
{
    double pi = 3.1415926535898;
    double thetax = theta_x*pi/180.0;

    Eigen::Matrix<double,3,3> Rx;
    Rx(0,0) = 1.0;
    Rx(0,1) = 0.0;
    Rx(0,2) = 0.0;
    Rx(1,0) = 0.0;
    Rx(1,1) = cos(thetax);
    Rx(1,2) = sin(thetax);
    Rx(2,0) = 0.0;
    Rx(2,1) = -sin(thetax);
    Rx(2,2) = cos(thetax);

    return Rx;
}

Eigen::Matrix<double,3,3> getRy(const double theta_y)
{
    double pi = 3.1415926535898;
    double thetay = theta_y*pi/180.0;

    Eigen::Matrix<double,3,3> Ry;
    Ry(0,0) = cos(thetay);
    Ry(0,1) = 0.0;
    Ry(0,2) = -sin(thetay);
    Ry(1,0) = 0.0;
    Ry(1,1) = 1.0;
    Ry(1,2) = 0.0;
    Ry(2,0) = sin(thetay);
    Ry(2,1) = 0.0;
    Ry(2,2) = cos(thetay);

    return Ry;
}

Eigen::Matrix<double,3,3> getRz(const double theta_z)
{
    double pi = 3.1415926535898;
    double thetaz = theta_z*pi/180.0;

    Eigen::Matrix<double,3,3> Rz;
    Rz(0,0) = cos(thetaz);
    Rz(0,1) = sin(thetaz);
    Rz(0,2) = 0.0;
    Rz(1,0) = -sin(thetaz);
    Rz(1,1) = cos(thetaz);
    Rz(1,2) = 0.0;
    Rz(2,0) = 0.0;
    Rz(2,1) = 0.0;
    Rz(2,2) = 1.0;
    return Rz;
}

Eigen::Matrix<double,3,3> getRyxz(const double thetax, const double thetay, const double thetaz)
{

    Eigen::Matrix<double,3,3> Ryxz;
    Ryxz =  getRy(thetay) * getRx(thetax) * getRz(thetaz);
    return Ryxz;
}


void calculcateGpsPoint(const double carGaussx,const double carGaussy,const double carGaussz,
                        const double carHeading,const double carPitch,const double carRoll,
                        const double signGaussx, const double signGaussy,const double signGaussz,
                        double& x, double& y,double& z)
{
    double Gaussx = signGaussx - carGaussx;
    double Gaussy = signGaussy - carGaussy;
    double Gaussz = signGaussz - carGaussz;

    Eigen::Matrix<double,3,3> Ryxz = getRyxz(carPitch, carRoll, -carHeading);// has problem
    Eigen::Matrix<double,3,1> point3d_vec;
    point3d_vec(0,0) = Gaussx;
    point3d_vec(1,0) = Gaussy;
    point3d_vec(2,0) = Gaussz;

    Eigen::Matrix<double,3,1> rtkDyDxDz = Ryxz * point3d_vec;

    x = rtkDyDxDz(0,0);
    y = rtkDyDxDz(1,0);
    z = rtkDyDxDz(2,0);
}

void backCalculcateGpsPoint(const double carGaussx,const double carGaussy,const double carGaussz,
                        const double carHeading,const double carPitch,const double carRoll,
                        const double x, const double y, const double z,
                        double& signGaussx, double& signGaussy, double& signGaussz)
{

    Eigen::Matrix<double,3,3> Ryxz = getRyxz(carPitch, carRoll, -carHeading);// has problem
//    Eigen::Matrix<double,3,3> inverseRyxz= Ryxz.inverse();
    
    Eigen::Matrix<double,3,1> point3d_vec;
    point3d_vec(0,0) = x;
    point3d_vec(1,0) = y;
    point3d_vec(2,0) = z;

    Eigen::Matrix<double,3,1> DyDxDz = (Ryxz.transpose()*Ryxz).inverse()*Ryxz.transpose() * point3d_vec;
//    Eigen::Matrix<double,3,1> DyDxDz = (Ryxz.transpose()*Ryxz).completeOrthogonalDecomposition().pseudoInverse()*Ryxz.transpose()* point3d_vec;
    signGaussx = DyDxDz(0,0) + carGaussx;
    signGaussy = DyDxDz(1,0) + carGaussy;
    signGaussz = DyDxDz(2,0) + carGaussz;
}



cv::Point3d getDXDYRefCamera(const cv::Point3d cameraPos, const cv::Point3d pose, const cv::Point3d roadPoint)
{
	double cameraPosEnuX, cameraPosEnuY, cameraPosEnuZ;
	double roadPointEnuX, roadPointEnuY, roadPointEnuZ;
	double refX, refY, refZ;
	conv84ToEnu(cameraPosEnuX, cameraPosEnuY, cameraPosEnuZ, cameraPos.x, cameraPos.y, 0.0);
	conv84ToEnu(roadPointEnuX, roadPointEnuY, roadPointEnuZ, roadPoint.x, roadPoint.y, 0.0);
	
	calculcateGpsPoint(cameraPosEnuX, cameraPosEnuY, cameraPosEnuZ,
                        pose.x,0.0,0.0,
                        roadPointEnuX, roadPointEnuY, roadPointEnuZ,
                        refX, refY, refZ);

	return cv::Point3d{refX, refY, refZ};
	
}

std::vector<cv::Point2d> transforEnu2LongLat(const std::vector<cv::Point2d> carsPosEnu)
{
	std::vector<cv::Point2d> retCarsPosLongLat;
	for(int i=0;i<carsPosEnu.size();i++)
	{
		double carsPosX, carsPosY, carsPosZ;
		convEnuTo84(carsPosX, carsPosY, carsPosZ, carsPosEnu[i].x, carsPosEnu[i].y, 0.0);
		retCarsPosLongLat.push_back(cv::Point2d{carsPosX, carsPosY});
		
	}
	return retCarsPosLongLat;
}

std::vector<cv::Point2d> transforRefCamera2LongLat(const std::vector<cv::Point2d> carsPosRefCamera, const cv::Point3d cameraPos, const cv::Point3d pose)
{

	double cameraPosEnuX, cameraPosEnuY, cameraPosEnuZ;
	conv84ToEnu(cameraPosEnuX, cameraPosEnuY, cameraPosEnuZ, cameraPos.x, cameraPos.y, 0.0);
	
	std::vector<cv::Point2d> carsPosEnu;
	for(int i=0;i<carsPosRefCamera.size();i++)
	{
		double carPosEnuX, carPosEnuY, carPosEnuZ;
		backCalculcateGpsPoint(cameraPosEnuX, cameraPosEnuY, cameraPosEnuZ,
                        pose.x,0.0,0.0,
                        carsPosRefCamera[i].x, carsPosRefCamera[i].y, 0.0,
                        carPosEnuX, carPosEnuY, carPosEnuZ);
		carsPosEnu.push_back(cv::Point2d{carPosEnuX, carPosEnuY});

	}	
	return transforEnu2LongLat(carsPosEnu);
	
}



std::vector<std::vector<cv::Point2d>> getRoadPointsDXDYRefCamera(const cv::Point3d cameraPos, const cv::Point3d pose, const std::vector<std::vector<cv::Point2d>> roadPoints)
{
	std::vector<std::vector<cv::Point2d>> retRoadPoints;
	for(int i=0; i<roadPoints.size(); i++)
	{
		std::vector<cv::Point2d> oneLane;
		for(int j=0; j<roadPoints[i].size(); j++)
		{
			cv::Point3d refCameraDXDYDZ = getDXDYRefCamera(cameraPos, pose, cv::Point3d{roadPoints[i][j].x, roadPoints[i][j].y, 0.0});
			oneLane.push_back(cv::Point2d{refCameraDXDYDZ.x, refCameraDXDYDZ.y});	
		}
		retRoadPoints.push_back(oneLane);
		
	}
	return retRoadPoints;
}
double getLRON(double x1,double y1, double x2, double y2, double x3, double y3)
{
	/*
	 * result<0 (x3,y3)在向量x1x2的右侧
	 * result>0 (x3,y3)在向量x1x2的左侧
	 * result=0 (x3,y3)在向量x1x2上
	 */
	double result = (x1-x3)*(y2-y3)-(y1-y3)*(x2-x3);
	return result;
}

std::vector<CARPOSINFO> getLaneIndAndOffSet(const std::vector<cv::Point2d> carsPos, const cv::Mat maskLane)
{
	std::vector<CARPOSINFO> retCarInLaneIndAndOffSet;
	for(int i=0;i<carsPos.size();i++)
	{
		CARPOSINFO carPosInfo = getLaneIndAndOffSet(carsPos[i].x, carsPos[i].y, maskLane);
		retCarInLaneIndAndOffSet.push_back(carPosInfo);
	}
	return retCarInLaneIndAndOffSet;
	
	
}

cv::Point2d getCarPosWithRefHdMap(const std::vector<std::vector<cv::Point2d>> laneRefCameraSequent, const cv::Point2d carRefCameraPos, const CARPOSINFO carPosInfo)
{
//	CARPOSINFO getLaneIndAndOffSet(const cv::Point2d carPos, const std::string maskFileName);
	
	std::vector<int> retNearPointsIndex;
	int lanePointCount = laneRefCameraSequent[carPosInfo.laneInd].size();
	int step = laneRefCameraSequent[carPosInfo.laneInd][0].y < laneRefCameraSequent[carPosInfo.laneInd][lanePointCount-1].y?1:-1;
	int startInd = 1==step?0:lanePointCount-1;
	double offSetXRefLeftLane;
	int upMinInd = -1, downMaxInd = -1;
	
	for(int i=startInd; i<lanePointCount || i<0; i+=step)
	{
		if(laneRefCameraSequent[carPosInfo.laneInd][i].y > carRefCameraPos.y)
		{
			upMinInd = i;
			break;
		}
	}
	
	downMaxInd = upMinInd-step;
	double d1 = carRefCameraPos.y - laneRefCameraSequent[carPosInfo.laneInd][downMaxInd].y;
	double rate = d1/(laneRefCameraSequent[carPosInfo.laneInd][upMinInd].y - laneRefCameraSequent[carPosInfo.laneInd][downMaxInd].y);
	double x1 = (1.0-rate)*laneRefCameraSequent[carPosInfo.laneInd][downMaxInd].x + rate*laneRefCameraSequent[carPosInfo.laneInd][upMinInd].x;
	double y1 = (1.0-rate)*laneRefCameraSequent[carPosInfo.laneInd][downMaxInd].y + rate*laneRefCameraSequent[carPosInfo.laneInd][upMinInd].y;
	
	double laneBetweenWidth=3.75;
	
	double x0 = laneRefCameraSequent[carPosInfo.laneInd][upMinInd].x;
	double y0 = laneRefCameraSequent[carPosInfo.laneInd][upMinInd].y;
	double offSetXD = laneBetweenWidth*carPosInfo.offSetXRefLeftLane;
	double middleYX = (y0-y1)/(x0-x1);
	double X0 = -1.0*middleYX * std::sqrt(offSetXD*offSetXD/(middleYX*middleYX+1.0)) +x1;
	double Y0 = std::sqrt(offSetXD*offSetXD/(middleYX*middleYX+1.0)) + y1;

	double X1 = -1.0*middleYX * -1.0*std::sqrt(offSetXD*offSetXD/(middleYX*middleYX+1.0)) +x1;
	double Y1 = -1.0*std::sqrt(offSetXD*offSetXD/(middleYX*middleYX+1.0)) + y1;
	
	if(getLRON(x1, y1, laneRefCameraSequent[carPosInfo.laneInd][upMinInd].x, laneRefCameraSequent[carPosInfo.laneInd][upMinInd].y, X0, Y0)<=0.0)
	{
		return cv::Point2d{X0, Y0};
		
	}else
	{
		return cv::Point2d{X1, Y1};
	}
}


std::vector<cv::Point2d> getCarsPosWithRefHdMap(const std::vector<std::vector<cv::Point2d>> laneRefCameraSequent, const std::vector<cv::Point2d> carsRefCameraPos, const std::vector<CARPOSINFO> carsPosInfo)
{
	std::vector<cv::Point2d> retCarsPosWithHdMap;
	for(int i=0;i<carsRefCameraPos.size();i++)
	{
		cv::Point2d carPosWithHdMap = getCarPosWithRefHdMap(laneRefCameraSequent, carsRefCameraPos[i], carsPosInfo[i]);
		retCarsPosWithHdMap.push_back(carPosWithHdMap);
	}
	return retCarsPosWithHdMap;	
}



std::vector<Eigen::Vector2d> transforCVPoints2EigenVector2d(const std::vector<cv::Point2d> points)
{
	std::vector<Eigen::Vector2d> retEigenV;
	for(int i=0;i<points.size();i++)
	{
		retEigenV.push_back(Eigen::Vector2d(points[i].x, points[i].y));
	}
	return retEigenV;
}

std::vector<std::vector<Eigen::Vector2d>> transforCVPoints2EigenVector2d(const std::vector<std::vector<cv::Point2d>> points)
{
	std::vector<std::vector<Eigen::Vector2d>> retEigenV;
	for(int i=0;i<points.size();i++)
	{
		retEigenV.push_back(transforCVPoints2EigenVector2d(points[i]));
	}
	return retEigenV;
}


std::vector<CARPOSINFO> getLaneIndAndOffSetRefHdMap(const std::vector<CARPOSINFO> carsPosInfo, const int laneCount)
{
	std::vector<CARPOSINFO> retCarsPosInfo;
	for(int i=0;i<carsPosInfo.size();i++)
	{
		if(carsPosInfo[i].laneInd<0)
		{
			retCarsPosInfo.push_back(carsPosInfo[i]);
			continue;
		}
		
		if(fabs(carsPosInfo[i].offSetXRefLeftLane)<0.0001)
		{
			CARPOSINFO oneCarPosInfo;
			oneCarPosInfo.laneInd = (laneCount-1) - carsPosInfo[i].laneInd;
			oneCarPosInfo.offSetXRefLeftLane = carsPosInfo[i].offSetXRefLeftLane;
			retCarsPosInfo.push_back(oneCarPosInfo);
		}else{
			CARPOSINFO oneCarPosInfo;
			oneCarPosInfo.laneInd = (laneCount-2) - carsPosInfo[i].laneInd;
			oneCarPosInfo.offSetXRefLeftLane = 1.0 - carsPosInfo[i].offSetXRefLeftLane;
			retCarsPosInfo.push_back(oneCarPosInfo);
		}				
	}
	return retCarsPosInfo;
}

std::vector<cv::Point2d> getCarDxDyRefhdMap(const std::vector<cv::Point2d> carDxDy)
{
	std::vector<cv::Point2d> retCarDxDy;
	for(int i=0;i<carDxDy.size();i++)
	{
		retCarDxDy.push_back(cv::Point2d{-1.0*carDxDy[i].x, -1.0*carDxDy[i].y});
	}
	return retCarDxDy;
}


void print2dPoints(const std::vector<std::vector<cv::Point2d>> laneSequent)
{
	for(int i=0;i<laneSequent.size();i++)
	{
		std::cout<<"-------------------------------------------lane Id "<<i<<" "<<laneSequent[i].size()<<std::endl;
		for(int j=0;j<laneSequent[i].size();j++)
		{
			std::cout<<laneSequent[i][j]<<std::endl;
		}
		
	}
}

bool writeRow(std::ofstream& outPutCsvFile, const int frameInd, const std::vector<CARDETECTED> carsDetectInfo, const std::vector<cv::Point2d> carsPosLongLat)
{
	std::string csvRow = std::to_string(frameInd);
	for(int i=0;i<carsPosLongLat.size();i++)
	{
		csvRow = csvRow + "," + std::to_string(carsDetectInfo[i].carId) + "," + to_string_with_precision(carsPosLongLat[i].x) + "," + to_string_with_precision(carsPosLongLat[i].y);
	}
	outPutCsvFile << csvRow <<std::endl;
	return true;
}

void getDataInRange(std::vector<cv::Point2d>& carDxDy, std::vector<CARDETECTED>& carDetectInfo, const double maxDis=200.0, const double minDis=-200.0)
{
	std::vector<CARDETECTED>::iterator itDetect= carDetectInfo.begin();
	for(std::vector<cv::Point2d>::iterator it= carDxDy.begin(); it!=carDxDy.end();)
	{
		if((*it).y>maxDis || (*it).y<minDis)
		{
			it=carDxDy.erase(it);
			itDetect = carDetectInfo.erase(itDetect);
		}
		else
		{
			++it;
			++itDetect;
		}
	}
	
}

void getDataInLanes(std::vector<cv::Point2d>& carDxDy, std::vector<CARDETECTED>& carDetectInfo, std::vector<CARPOSINFO>& carsPosInfo)
{
	std::vector<cv::Point2d>::iterator itDxDy= carDxDy.begin();
	std::vector<CARDETECTED>::iterator itDetect= carDetectInfo.begin();
	for(std::vector<CARPOSINFO>::iterator it= carsPosInfo.begin(); it!=carsPosInfo.end();)
	{
		if((*it).laneInd<0)
		{
			it = carsPosInfo.erase(it);
			itDxDy = carDxDy.erase(itDxDy);
			itDetect = carDetectInfo.erase(itDetect);
		}
		else
		{
			++it;
			++itDxDy;
			++itDetect;
		}
	}
	
}


const char * usage = 
"\n"
"./testGetCarTrackXY -config=../data/defaultConfigNan.yml"
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
	std::cout.precision(13);
	
	std::string configFile;
	getParserInfo(configFile, argc, argv);
	std::cout<< "configFile:"<<configFile<<std::endl;

	
	cv::Mat P;
	cv::Point3d cameraPos;
	cv::Point2d crossPoint;
	double cameraPitch;
	double cameraHeigth;
	double roadHeading;
	std::string videoName;
	std::string maskFileName;
	std::string hdmapName;
	std::string carDataName;
	std::string intrinsicName;
	std::string writeCarTrackName;
	bool ret = getConf(P, cameraPos, crossPoint, roadHeading, cameraPitch, cameraHeigth, videoName, maskFileName, intrinsicName, hdmapName, carDataName, writeCarTrackName, configFile);
	std::cout<<"---------------------------------------------------------"<<std::endl;
	std::cout<<"videoName: "<<std::endl<<videoName<<std::endl;
	std::cout<<"maskFileName: "<<std::endl<<maskFileName<<std::endl;
	std::cout<<"intrinsicName: "<<std::endl<<intrinsicName<<std::endl;
	std::cout<<"writeCarTrackName: "<<writeCarTrackName<<std::endl;
	std::cout<<"hdmapName: "<<std::endl<<hdmapName<<std::endl;
	std::cout<<"carDataName: "<<std::endl<<carDataName<<std::endl;
	std::cout<<"cameraPos: "<<std::endl<<cameraPos<<std::endl;
	std::cout<<"cameraPitch: "<<std::endl<<cameraPitch<<std::endl;
	std::cout<<"cameraHeigth: "<<std::endl<<cameraHeigth<<std::endl;
	std::cout<<"crossPoint: "<<std::endl<<crossPoint<<std::endl;
	std::cout<<"roadHeading: "<<std::endl<<roadHeading<<std::endl;
	std::cout<<"---------------------------------------------------------"<<std::endl;
	
	std::ofstream outPutCsvFile(writeCarTrackName);
	cv::Mat maskLane = cv::imread(maskFileName, cv::IMREAD_UNCHANGED);
	std::vector<std::vector<cv::Point2d>> laneSequent = getLaneSequent(hdmapName);
	std::vector<std::vector<cv::Point2d>> laneRefCameraSequent = getRoadPointsDXDYRefCamera(cameraPos, cv::Point3d{roadHeading, 0.0, 0.0}, laneSequent);
//	print2dPoints(laneRefCameraSequent);
	std::cout << "P: "<<P<<std::endl;
	
	for(int i=0;i<laneRefCameraSequent.size();i++)
	{
		std::cout<<"------------- "<<i<<std::endl;
		for(int j=0;j<laneRefCameraSequent[i].size();j++)
		{
			std::cout<<laneRefCameraSequent[i][j]<<std::endl;
		}
	}
	
	
	
	std::ifstream carPosCsvFile(carDataName);
	std::vector<CARDETECTED> carDetectInfo;
	int frameIndex = -1;
	
	while(getCarDetect(carDetectInfo, carPosCsvFile))
	{
		
		clock_t startTime,endTime;
//		startTime = clock();
		frameIndex++;
		std::cout<<"frameIndex: "<<frameIndex<<std::endl;
#ifdef ZHAODEBUG
		if(frameIndex<174)
		{
			continue;
		}
#endif
		std::vector<cv::Point2d> carsPosLongLat;
		std::vector<cv::Point2d> carDxDy;
		std::vector<CARPOSINFO> carsPosInfo;
		
		if(!carDetectInfo.empty())
		{
			withGeometry::TiLane laneDxDy;
			carDxDy = laneDxDy.getPointSetsXY(extractCarMidPoint(carDetectInfo), crossPoint, P, cameraHeigth, cameraPitch);
			getDataInRange(carDxDy, carDetectInfo);
			carsPosInfo = getLaneIndAndOffSet(extractCarMidPoint(carDetectInfo), maskLane);
			getDataInLanes(carDxDy, carDetectInfo, carsPosInfo);
		}
		
		
		if(!carDetectInfo.empty() && !carDxDy.empty() && carDetectInfo.size() == carDxDy.size())
		{	
			if(configFile.npos != configFile.find("Bei"))
			{
				int beiRoadLaneCount=4;
				carsPosInfo = getLaneIndAndOffSetRefHdMap(carsPosInfo, beiRoadLaneCount);
				carDxDy = getCarDxDyRefhdMap(carDxDy);
			}
			std::vector<cv::Point2d> debugMidPoints = extractCarMidPoint(carDetectInfo);
#ifdef ZHAODEBUG
			for(int i=0;i<carsPosInfo.size();i++)
			{
				std::cout<<"laneId offsetX middlePoint carId, dxdy: "<<carsPosInfo[i].laneInd<<" "<<carsPosInfo[i].offSetXRefLeftLane<<" , "<<debugMidPoints[i]<<" , "<<carDetectInfo[i].carId<<" , "<<carDxDy[i]<<std::endl;
			}
#endif
			std::vector<cv::Point2d> carsPosWithHdMap = getCarsPosWithRefHdMap(laneRefCameraSequent, carDxDy, carsPosInfo);
			carsPosLongLat = transforRefCamera2LongLat(carsPosWithHdMap, cameraPos, cv::Point3d{roadHeading, 0.0, 0.0});
#ifdef ZHAODEBUG
			if(frameIndex>174)
			{
				showCarTrack(transforCVPoints2EigenVector2d(laneRefCameraSequent), transforCVPoints2EigenVector2d(carsPosWithHdMap));
			}
#endif
//			showCarTrack(transforCVPoints2EigenVector2d(laneRefCameraSequent), transforCVPoints2EigenVector2d(carsPosWithHdMap));
//			
		}
		
		writeRow(outPutCsvFile, frameIndex, carDetectInfo, carsPosLongLat);
//		endTime = clock();
//		std::cout << "The run time is:" <<(double)(endTime - startTime)*1000 / CLOCKS_PER_SEC << "ms" << std::endl;
	}

	
	outPutCsvFile.close();
	carPosCsvFile.close();
	return 0;
}
