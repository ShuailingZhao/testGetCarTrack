
#include <opencv2/opencv.hpp>
#include <vector>
#include "getCarInWitchLane.h"
//using namespace cv;
//using namespace std;

CARPOSINFO getLaneIndAndOffSet(const int x, const int y, const cv::Mat label)
{

    CARPOSINFO retCarPosInfo;
    retCarPosInfo.laneInd = 0;
    //Mat img = imread("../image/1.jpeg");
//    cv::Mat label = cv::imread(maskFileName, cv::IMREAD_UNCHANGED);
    cv::Mat img_gray;
    cv::cvtColor(label, img_gray, cv::COLOR_BGR2GRAY);
    img_gray.convertTo(img_gray, CV_32FC1);
    int w = label.cols;
    int h = label.rows;
    std::vector<int> index;
    
    if(x >= w or y >= h)
    {
        retCarPosInfo.laneInd = -1;
        retCarPosInfo.offSetXRefLeftLane = -1;
    }
    else if(x == w - 1)
    {
        for(int i = 0; i <= x; ++i)
        {

            if(img_gray.at<float>(y, i) != 255 && img_gray.at<float>(y, i+1) == 255)
            {
                retCarPosInfo.laneInd++;
                index.push_back(i);
            }
        }
        double left_dist = x - index.back();
        double dist_mean = (index[2] - index[0]) / 3;
        retCarPosInfo.laneInd = 4;
        retCarPosInfo.offSetXRefLeftLane = left_dist / dist_mean;
    }
    else {
        for(int i = 0; i <= x; ++i)
        {
            if(img_gray.at<float>(y, i) != 255 && img_gray.at<float>(y, i+1) == 255)
            {
                retCarPosInfo.laneInd++;
                index.push_back(i);
            }
        }
        
        if(index.size() == 0)
        {
            retCarPosInfo.laneInd = -1;
            retCarPosInfo.offSetXRefLeftLane = -1;
        }   
        else
        {
            double left_dist = x - index.back();
            std::vector<int> index_right;
            for(int i = x+1; i <= w; ++i)
            {
                if(img_gray.at<float>(y, i) != 255 && img_gray.at<float>(y, i+1) == 255)
                {
                    index_right.push_back(i);
                }
            }
            if(index_right.size() == 0)
            {
                double dist_mean = (index[2] - index[0]) / 3;
                retCarPosInfo.offSetXRefLeftLane = left_dist / dist_mean;
            }
            else
            {
                retCarPosInfo.offSetXRefLeftLane = left_dist / (index_right[0] - index.back());
            }
        }

    }
    if(retCarPosInfo.laneInd>0)
    {
	retCarPosInfo.laneInd-=1;
    }
	
//    cv::circle(label, cv::Point2i(x, y), 6, cv::Scalar(255, 0, 0), -1, 16);
//    cv::namedWindow("test", 0);
//    cv::imshow("test", label);
//    cv::waitKey(0);
    //imwrite("aa.png", label);

    return retCarPosInfo;
}
