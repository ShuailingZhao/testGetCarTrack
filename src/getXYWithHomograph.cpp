#include "getXYWithHomograph.h"
namespace withHomograph{
	cv::Point2f transferPoint(cv::Point2f point)
	{
		cv::Size framesize(1280,720);
		cv::Size newFramesize(256,144);
		cv::Point2f newPoint; 
		point.y = point.y - framesize.height * 0.5;
		double scaleX = newFramesize.width / (framesize.width * 1.0);
		double scaleY = newFramesize.height / (framesize.height * 0.5);
		newPoint.x = point.x * scaleX;
		newPoint.y = point.y * scaleY;
		return newPoint;
	}

	cv::Point2f getDisPoint(cv::Rect rect)
	{
		cv::Size imgSize(1280,720);
		cv::Point2f upPoint, downPoint;
		upPoint.x = rect.x + rect.width/2.0;
		upPoint.y = rect.y;
		downPoint.x = rect.x + rect.width/2.0;
		downPoint.y = rect.y + rect.height;

	
		if (imgSize.height - downPoint.y > 40)// lager num means higher downPoint
		{//Using downPoint to get distance
			return downPoint;
		}else
		{//using upPoint to get distance
			return upPoint;
		}
	}
	cv::Mat getDis(cv::Mat m, cv::Mat points)
	{
		return m * points;
	}

	void getRealDyDx(cv::Point2f p, double& dy,double& dx)
	{
		//dy = 32.0 - p.y + 6.1;
		//dx = p.x - 10.0 - 0.21;
		dy = 32.0 - p.y + 1.0;
		dx = p.x - 11.0 + 0.03;
	}
	cv::Point2d getDyDx(cv::Mat m, cv::Point2f point)
	{
		double dy, dx;
		cv::Mat points(3,1,CV_64F);
	
	    //1.Get perspective image
		points.at<double>(0,0) = point.x;	
		points.at<double>(1,0) = point.y;	
		points.at<double>(2,0) = 1;	
		cv::Mat dis = getDis(m,points);

		dis.at<double>(0,0) = dis.at<double>(0,0)/dis.at<double>(2,0);	
		dis.at<double>(1,0) = dis.at<double>(1,0)/dis.at<double>(2,0);	
	    	dis.at<double>(2,0) = dis.at<double>(2,0)/dis.at<double>(2,0);

	    //2.Coordinate transformation
		cv::Point2f poi;
	    	poi.x = dis.at<double>(0,0)/10; // convert birdView image to real distance(metric m)
		poi.y = dis.at<double>(1,0)/10;
		getRealDyDx(poi,dy,dx);
		return cv::Point2d(dx,dy);
	    
	}

}
