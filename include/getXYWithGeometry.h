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
#include "bA.h"
namespace withGeometry
{
	struct SignRefInfo{
		std::vector<cv::Point2d> signCoordinate;
		double H2;
		double h2s;
		double h2;
	};
	
	struct SignCorner{
		std::vector<cv::Point2d> corner;
	};
	
	
	class TiLane{
	public:
		TiLane();
//		std::vector<cv::Point2d> getPointsXY(const std::vector<cv::Point2d> pts, const cv::Mat K, const double h);
//		cv::Point2d hFactorModel(const cv::Point2d pt, const cv::Mat K, double h);
//		cv::Point2d hPitchRollFactorsModel(const cv::Point2d pt, const cv::Mat K, double h);
		std::vector<cv::Point2d> getPointSetsXY(const std::vector<cv::Point2d> pts, const cv::Point2d crossPoint, const cv::Mat K, const double h, const double pitch);
		cv::Point2d getPointXY(const cv::Point2d pt, const cv::Point2d crossPoint, const cv::Mat K, const double h, const double pitch);
		cv::Point2d getCrossPoint(const std::vector<cv::Point2d>lines);
		double getPitch(const cv::Mat cameraMatrix, const cv::Point2d vanishPoint);
	private:
	};
	
	class TiSign{
	public:
		TiSign();
		SignCorner getPointXY(const std::vector<double> steps, const std::vector<SignCorner> signInfo,  const cv::Point2d crossPoint, const cv::Mat K);
		std::vector<cv::Point2d> transforSignCorner2Points(const SignCorner sign);
		SignRefInfo getSignInfo(const double step, const SignCorner currentSign, const SignCorner beforeStepSign, const cv::Point2d crossPoint, const cv::Mat K);
		double getMeanD(const std::vector<SignRefInfo> signMidInfoSets);
		double getMeanH2(const std::vector<SignRefInfo> signMidInfoSets);
		std::vector<double> geth2Sets(const std::vector<SignRefInfo> signMidInfoSets);
	private:
	};
	
	class TiPole{
	public:
		TiPole();
		cv::Point2d getPointXY(const double step, const cv::Point2d polePoint, const cv::Point2d polePointBeforeStep, const cv::Point2d crossPoint, const cv::Mat K);
	private:
	};

}
#endif


