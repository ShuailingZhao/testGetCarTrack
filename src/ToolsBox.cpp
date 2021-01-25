#include "ToolsBox.h"
#include <sstream>





bool drawSamplePoints(cv::Mat& img, std::vector<std::vector<Eigen::Vector2d>> iLanes2DBack, double iScale, cv::Scalar iColor, int iRadius, int iThickness)
{

    // 70m x 200m
    //img is 210x600,,1/iScalem p pixel

    // m->pixel m*iScale; pixel->m pixel/iScale
    //m->img pipelane
    //y  O.y - (y)m->pixel  图像纵坐标 O.y - y*iScale
    //x  O.x + (x)m->pixel  图像横坐标 O.x + x*iScale

    int w,h;

    w = img.cols;
    h = img.rows;
    cv::Point2d O(int(w/2.0),int(h*3.0/4.0));//50m
    cv::circle(img, cv::Point(O.x,O.y), 4, cv::Scalar(0,0,0),-1);

    for(int i=0;i<iLanes2DBack.size();i++)
//    for(int i=2;i<3;i++)
    {
        for(int j=0;j<iLanes2DBack[i].size();j++)
        {
            double x,y;

            x = iLanes2DBack[i][j](1,0);
            y = iLanes2DBack[i][j](0,0);
            int imgX,imgY;
            imgX = O.x + x*iScale;
            imgY = O.y - y*iScale;

            cv::circle(img, cv::Point(imgX,imgY),iRadius, iColor,iThickness);


        }
    }
    return true;
}

bool drawRoadLane(cv::Mat& img, std::vector<std::vector<Eigen::Vector2d>> iLanes2DBack, double iScale, cv::Scalar iColor, int iRadius, int iThickness)
{

    // 70m x 200m
    //img is 210x600,,1/iScalem p pixel

    // m->pixel m*iScale; pixel->m pixel/iScale
    //m->img pipelane
    //y  O.y - (y)m->pixel  图像纵坐标 O.y - y*iScale
    //x  O.x + (x)m->pixel  图像横坐标 O.x + x*iScale

    int w,h;

    w = img.cols;
    h = img.rows;
    cv::Point2d O(int(w/2.0),int(h*3.0/4.0));//50m
    cv::circle(img, cv::Point(O.x,O.y), 4, cv::Scalar(0,0,0),-1);

    for(int i=0;i<iLanes2DBack.size();i++)
    {
        for(int j=0;j<iLanes2DBack[i].size()-1;j++)
        {
            double x0, y0, x1, y1;

            x0 = iLanes2DBack[i][j](0,0);
            y0 = iLanes2DBack[i][j](1,0);
            int imgX0,imgY0;
            imgX0 = O.x + x0*iScale;
            imgY0 = O.y - y0*iScale;

	    x1 = iLanes2DBack[i][j+1](0,0);
            y1 = iLanes2DBack[i][j+1](1,0);
            int imgX1,imgY1;
            imgX1 = O.x + x1*iScale;
            imgY1 = O.y - y1*iScale;

            
            cv::line(img,cv::Point(imgX0,imgY0), cv::Point(imgX1,imgY1),iColor,iRadius);

//            cv::circle(img, cv::Point(imgX,imgY),iRadius, iColor,iThickness);


        }
    }
    return true;
}


bool drawFittingLane(cv::Mat& img, std::vector<LANE> ivLane, double iScale, cv::Scalar iColor, int iRadius, int iThickness)
{
    // 70m x 200m
    //img is 70*iScalex200*iScale,,1/iScale m every pixel | 1 m iScale pixel

    // m->pixel m*iScale; pixel->m pixel/iScale
    //m->img pipelane
    //y  O.y - (y)m->pixel  图像纵坐标 O.y - y*iScale
    //x  O.x + (x)m->pixel  图像横坐标 O.x + x*iScale

    int w,h;

    w = img.cols;
    h = img.rows;
//    cv::Point2d O(int(w/2.0),int(h*3.0/4.0));//50m
    cv::Point2d O(int(w/2.0),int(h*3.0/4.0));//50m
    cv::circle(img, cv::Point(O.x,O.y), 2, cv::Scalar(0,0,0),-1);

    for(int i=0;i<ivLane.size();i++)
//    for(int i=2;i<3;i++)
    {
        double y;//横坐标
        double x;
        for(x=-10.0;x<70.0;x+=0.3)//纵坐标
        {
            y=ivLane[i].a0 + ivLane[i].a1*x + ivLane[i].a2*x*x + ivLane[i].a3*x*x*x;

            int imgX,imgY;
            imgX = O.x + y*iScale;
            imgY = O.y - x*iScale;

            cv::circle(img, cv::Point(imgX,imgY), iRadius, iColor,iThickness);
        }


    }

    return true;

}
void showLanes(const std::vector<LANE> fittingLanes, const std::vector<std::vector<Eigen::Vector2d>> validRangePoints2D, const std::vector<std::vector<Eigen::Vector2d>> lanesAllPoints2D)
{
	std::vector<int> selectIndex={};
	
	int stopIndex=1000000;
	static int frameIndex=1;
	frameIndex++;
	cv::namedWindow("debugWin");
        double scale=8.0;//default 10.0//窗口的大小
	double figureScale=7.0;//default 10.0图像的大小
        cv::Scalar redColor=cv::Scalar(0,0,255);
        cv::Scalar greenColor=cv::Scalar(0,255,0);
        cv::Scalar blueColor=cv::Scalar(255,0,0);
        cv::Mat imshowImg(100*scale,100*scale,CV_8UC3,cv::Scalar(255,255,255));
	drawIndex(imshowImg, std::to_string(frameIndex));
        drawFittingLane(imshowImg, selectFittingLanes(fittingLanes,selectIndex), figureScale, redColor);
        drawSamplePoints(imshowImg, selectLanePoints(lanesAllPoints2D,selectIndex), figureScale, blueColor);
        drawSamplePoints(imshowImg, selectLanePoints(validRangePoints2D,selectIndex), figureScale,greenColor,4,1);
        cv::imshow("debugWin",imshowImg);
        if(frameIndex>stopIndex)
        {
        	cv::waitKey(0);
        }
        cv::waitKey(0);
	
}


bool drawCarPos(cv::Mat& img, std::vector<Eigen::Vector2d> carsPos, double iScale, cv::Scalar iColor, int iRadius, int iThickness)
{

    // 70m x 200m
    //img is 210x600,,1/iScalem p pixel

    // m->pixel m*iScale; pixel->m pixel/iScale
    //m->img pipelane
    //y  O.y - (y)m->pixel  图像纵坐标 O.y - y*iScale
    //x  O.x + (x)m->pixel  图像横坐标 O.x + x*iScale

    int w,h;

    w = img.cols;
    h = img.rows;
    cv::Point2d O(int(w/2.0),int(h*3.0/4.0));//50m
//    cv::circle(img, cv::Point(O.x,O.y), 4, cv::Scalar(0,0,0),-1);

    for(int i=0;i<carsPos.size();i++)
    {
        
	double x,y;
	x = carsPos[i](0,0);
	y = carsPos[i](1,0);
	int imgX,imgY;
	imgX = O.x + x*iScale;
	imgY = O.y - y*iScale;

	cv::circle(img, cv::Point(imgX,imgY),iRadius, iColor,iThickness);


    }
    return true;
}


void showCarTrack(const std::vector<std::vector<Eigen::Vector2d>> laneSequent, const std::vector<Eigen::Vector2d> carsPos)
{
	std::vector<int> selectIndex={};
	
	int stopIndex=1000000;
	static int frameIndex=1;
	frameIndex++;
	cv::namedWindow("debugWin");
        double scale=9.0;//default 10.0//窗口的大小
	double figureScale=3.0;//default 10.0图像的大小
        cv::Scalar redColor=cv::Scalar(0,0,255);
        cv::Scalar greenColor=cv::Scalar(0,255,0);
        cv::Scalar blueColor=cv::Scalar(255,0,0);
        cv::Mat imshowImg(100*scale,100*scale,CV_8UC3,cv::Scalar(255,255,255));
        drawCarPos(imshowImg, carsPos, figureScale, greenColor, 4, 1);
        drawRoadLane(imshowImg, laneSequent, figureScale, blueColor);
        cv::imshow("debugWin",imshowImg);
        if(frameIndex>stopIndex)
        {
        	cv::waitKey(0);
        }
        cv::waitKey(0);
	
}

std::vector<LANE> selectFittingLanes(const std::vector<LANE> fittingLanes, const std::vector<int> selectIndex)
{
	if(selectIndex.empty())
	{
		return fittingLanes;
	}
	
	std::vector<LANE> selectFittingLanes;
	if(1 == selectIndex.size())
	{
		selectFittingLanes.push_back(fittingLanes[selectIndex[0]]);
		return selectFittingLanes;
	}
	
	selectFittingLanes.assign(fittingLanes.begin()+selectIndex[0],fittingLanes.begin()+selectIndex[1]+1);
	return selectFittingLanes;
}

std::vector<std::vector<Eigen::Vector2d>> selectLanePoints(const std::vector<std::vector<Eigen::Vector2d>> allLanes, const std::vector<int> selectIndex)
{
	if(selectIndex.empty())
	{
		return allLanes;
	}
	
	std::vector<std::vector<Eigen::Vector2d>> selectLanePoints;
	if(1 == selectIndex.size())
	{
		selectLanePoints.push_back(allLanes[selectIndex[0]]);
		return selectLanePoints;
	}
	
	selectLanePoints.assign(allLanes.begin()+selectIndex[0],allLanes.begin()+selectIndex[1]+1);
	return selectLanePoints;
}

void drawIndex(cv::Mat& img, const std::string windowName)
{
	cv::putText(img, windowName, cv::Point(10, 20), CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,0), 1, 8);

}

