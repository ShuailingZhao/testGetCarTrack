#ifndef ZHAOSL_TOOLSBOX_H
#define ZHAOSL_TOOLSBOX_H

#ifdef __INT16_TYPE__
typedef __INT16_TYPE__ int16_t;
#endif

#ifdef __INT64_TYPE__
typedef __INT64_TYPE__ int64_t;
#endif

#define IBS(n) (0x01<<(n-1))
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <dirent.h>
#include <sys/stat.h>
#include "CSVRow.h"
//#include "v_location.h"
#ifdef _WIN32
#include <Windows.h>
#include <strsafe.h>
#else
#include <dirent.h>
#endif
//#include "hdmapWrapper.h"
#include "fusionCommonData.h"
#include "wgs84_3degreeGauss.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>



bool drawFittingLane(cv::Mat& img, std::vector<LANE> ivLane, double iScale, cv::Scalar iColor=cv::Scalar(255,0,0), int iRadius=1, int iThickness=-1);
bool drawSamplePoints(cv::Mat& img, std::vector<std::vector<Eigen::Vector2d>> iLanes2DBack, double iScale, cv::Scalar iColor=cv::Scalar(0,0,255), int iRadius=2, int iThickness=-1);
bool drawRoadLane(cv::Mat& img, std::vector<std::vector<Eigen::Vector2d>> iLanes2DBack, double iScale, cv::Scalar iColor=cv::Scalar(0,0,255), int iRadius=1, int iThickness=-1);
bool drawCarPos(cv::Mat& img, std::vector<Eigen::Vector2d> carsPos, double iScale, cv::Scalar iColor, int iRadius=1, int iThickness=-1);

void showLanes(const std::vector<LANE> fittingLanes, const std::vector<std::vector<Eigen::Vector2d>> validRangePoints2D, const std::vector<std::vector<Eigen::Vector2d>> lanesAllPoints2D);

void showCarTrack(const std::vector<std::vector<Eigen::Vector2d>> laneSequent, const std::vector<Eigen::Vector2d> carsPos);

std::vector<LANE> selectFittingLanes(const std::vector<LANE> fittingLanes, const std::vector<int> selectIndex);

std::vector<std::vector<Eigen::Vector2d>> selectLanePoints(const std::vector<std::vector<Eigen::Vector2d>> allLanes, const std::vector<int> selectIndex);

void drawIndex(cv::Mat& img, const std::string windowName);

#endif
