#include "getXYWithGeometry.h"
namespace withGeometry{
	TiLane::TiLane()
	{}

	std::vector<cv::Point2d> TiLane::getPointSetsXY(const std::vector<cv::Point2d> pts, const cv::Point2d crossPoint, const cv::Mat K, const double h, const double pitch)
	{
		std::vector<cv::Point2d> ptsDxDy;
		for(int i=0;i<pts.size();i++)
		{
//			cv::Point2d pDxDy = hFactorModel(pts[i], K, h);
			cv::Point2d pDxDy = getPointXY(pts[i], crossPoint, K, h, pitch);
			ptsDxDy.push_back(pDxDy);
		}
		return ptsDxDy;	
	}
	
	cv::Point2d TiLane::getPointXY(const cv::Point2d pt, const cv::Point2d crossPoint, const cv::Mat K, const double h, const double pitch)
	{
	    double theta = pitch*3.1415926/180.0;
	    double dy = h * K.at<double>(1, 1) / (pt.y - crossPoint.y)/(cos(theta)*cos(theta)) - h*tan(theta);
	    double dx = h * (pt.x-K.at<double>(0, 2))/cos(theta)/(pt.y - crossPoint.y);
	    return cv::Point2d(dx, dy);
	}
	
//	cv::Point2d TiLane::hFactorModel(const cv::Point2d pt, const cv::Mat K, double h)
//	{
//	    double e = 1e-6;
//	    double dy = h * K.at<double>(1, 1) / ( (pt.y - K.at<double>(1, 2)) + e);
//	    double dx = dy * (pt.x - K.at<double>(0, 2)) / K.at<double>(0, 0);
//	    return cv::Point2d(dx, dy);
//	}

//	cv::Point2d TiLane::hPitchRollFactorsModel(const cv::Point2d pt, const cv::Mat K, double h)
//	{
//	    double thet = std::atan((pt.y - K.at<double>(1, 2)) / K.at<double>(1, 1)); //roll
//	    double gamma =std::atan((pt.x - K.at<double>(0, 2)) / K.at<double>(0, 0)); //pitch
//	    double dy = h / tan(thet);
//	    double dx = tan(gamma) * dy / cos(thet);
//	    return cv::Point2d(dx, dy);
//	}
	
	cv::Point2d TiLane::getCrossPoint(const std::vector<cv::Point2d>lines)
	{
		cv::Point2d crossPoint;
		float L1x0 = lines[0].x;
		float L1y0 = lines[0].y;
		float L1x1 = lines[1].x;
		float L1y1 = lines[1].y;
		float L3x0 = lines[2].x;
		float L3y0 = lines[2].y;
		float L3x1 = lines[3].x;
		float L3y1 = lines[3].y;


		float a1 = L1y0 - L1y1;
		float b1 = L1x1 - L1x0;
		float c1 = L1x0*L1y1 - L1x1*L1y0;
		float a3 = L3y0 - L3y1;
		float b3 = L3x1 - L3x0;
		float c3 = L3x0*L3y1 - L3x1*L3y0;
		float D = a1*b3 - a3*b1;
		cv::Point2d tmp_pt;
		if(abs(0.0-D)<0.00001)
		{
			tmp_pt.x=0.0;
			tmp_pt.y=0.0;
		}

		double x = (b1*c3 - b3*c1) / D;
		double y = (a3*c1 - a1*c3) / D;
		tmp_pt.x = x;
		tmp_pt.y = y;
		crossPoint = tmp_pt;
		return crossPoint;
	}
	
	double TiLane::getPitch(const cv::Mat cameraMatrix, const cv::Point2d vanishPoint)
	{
		double pitch = std::atan((cameraMatrix.at<double>(1, 2) - vanishPoint.y)/cameraMatrix.at<double>(1, 1))*180.0/3.1415926;//The value will be positive if camera looks down and negative looks up.
		return pitch;
	}
	
	TiSign::TiSign()
	{}
	
	SignCorner TiSign::getPointXY(const std::vector<double> steps, const std::vector<SignCorner> signInfo,  const cv::Point2d crossPoint, const cv::Mat K)
	{
		SignCorner retSignCorner;
		std::vector<cv::Point2d> retSignPoints;
		std::vector<double> sumSteps = bA::sumStep(steps);
		std::vector<SignRefInfo> signMidInfoSets;
		for(int i=0;i<sumSteps.size();i++)
		{
			SignRefInfo signMidInfo = getSignInfo(sumSteps[i], signInfo[signInfo.size()-1], signInfo[i], crossPoint, K);
			signMidInfoSets.push_back(signMidInfo);
		}
		
		double d = getMeanD(signMidInfoSets);
		double H2 = getMeanH2(signMidInfoSets);
		std::vector<double> h2Sets = geth2Sets(signMidInfoSets);
		double fy = K.at<double>(1, 1);
		bA::optimizeDAndH(d, H2, h2Sets, steps, fy);
		
		double mainPointY = K.at<double>(1, 2);
		double mainPointX = K.at<double>(0, 2);
		double v = mainPointY - crossPoint.y;
		double sqrtFV = sqrt(fy*fy+v*v);
		std::vector<cv::Point2d> signPoints = signInfo[signInfo.size()-1].corner;
		for(int i=0;i<signPoints.size();i++)
		{
			double py = mainPointY - signPoints[i].y;
			double px = signPoints[i].x - mainPointX;
			double X = d*px*sqrtFV/(fy*fy + py*v);
			retSignPoints.push_back(cv::Point2d(X,d));
		}
		retSignCorner.corner.assign(retSignPoints.begin(),retSignPoints.end());
		return retSignCorner;
	}
	
	std::vector<cv::Point2d> TiSign::transforSignCorner2Points(const SignCorner sign)
	{
		return sign.corner;
	}
	
	double TiSign::getMeanD(const std::vector<SignRefInfo> signMidInfoSets)
	{
		double d = 0.0;
		for(int i=0;i<signMidInfoSets.size();i++)
		{
			for(int j=0;j<signMidInfoSets[i].signCoordinate.size();j++)
			{
				d += signMidInfoSets[i].signCoordinate[j].y;
			}
		}
		return d/signMidInfoSets.size()/signMidInfoSets[0].signCoordinate.size();
	}
	
	double TiSign::getMeanH2(const std::vector<SignRefInfo> signMidInfoSets)
	{
		double H2 = 0.0;
		for(int i=0;i<signMidInfoSets.size();i++)
		{
			H2 += signMidInfoSets[i].H2;
		}
		return H2/signMidInfoSets.size();
	}
	std::vector<double> TiSign::geth2Sets(const std::vector<SignRefInfo> signMidInfoSets)
	{
		std::vector<double> h2Sets;
		for(int i=0;i<signMidInfoSets.size();i++)
		{
			h2Sets.push_back(signMidInfoSets[i].h2s);
		}
		h2Sets.push_back(signMidInfoSets[signMidInfoSets.size()-1].h2);
		return h2Sets;
	}
	
	SignRefInfo TiSign::getSignInfo(const double step, const SignCorner currentSign, const SignCorner beforeStepSign, const cv::Point2d crossPoint, const cv::Mat K)
	{
		std::vector<cv::Point2d> signPoints(currentSign.corner);
		std::vector<cv::Point2d> signPointsBeforeStep(beforeStepSign.corner);
		std::vector<cv::Point2d> signPointsCoordinate;
		double h1t, h1b, v, f, h1st, h1sb;
		f = K.at<double>(1, 1);
		double mainPointY = K.at<double>(1, 2);
		double mainPointX = K.at<double>(0, 2);
		v = mainPointY - crossPoint.y;
		h1t = mainPointY - signPoints[0].y;
		h1b = mainPointY - signPoints[1].y;
		h1st = mainPointY - signPointsBeforeStep[0].y;
		h1sb = mainPointY - signPointsBeforeStep[1].y;
		
		double h2 = (h1t - h1b)/(f*f + h1t*v)/(f*f + h1b*v);
		double h2s = (h1st - h1sb)/(f*f + h1st*v)/(f*f + h1sb*v);
		double d = step*h2s/(h2-h2s);
		double px,py, sqrtFV;
		sqrtFV = sqrt(f*f+v*v);
		for(int i=0;i<signPoints.size();i++)
		{
			py = mainPointY - signPoints[i].y;
			px = signPoints[i].x - mainPointX;
			double X = d*px*sqrtFV/(f*f + py*v);
			signPointsCoordinate.push_back(cv::Point2d(X,d));
		}
		
		SignRefInfo reSignRefInfo;
		reSignRefInfo.signCoordinate.assign(signPointsCoordinate.begin(), signPointsCoordinate.end());
		
		double fai1 = atan(h1t/f);
		double fai2 = atan(h1b/f);
		double theta = atan(v/f);
		h2 = f*tan(fai1-theta)-f*tan(fai2-theta);
		double fai1s = atan(h1st/f);
		double fai2s = atan(h1sb/f);
		h2s = f*tan(fai1s-theta)-f*tan(fai2s-theta);
		d = step * h2s/(h2-h2s);
		double H2 = step*h2*h2s/f/(h2-h2s);
//		std::cout<<"f: "<<f<<std::endl;
//		std::cout<<"h1: "<<h1b-h1t<<" h1s: "<<h1sb-h1st<<std::endl;
//		std::cout<<"h2: "<<h2<<" h2s:"<<h2s<<" step: "<<step<<" H2: "<<H2<<std::endl;
		reSignRefInfo.H2 = H2;
		reSignRefInfo.h2s = h2s;
		reSignRefInfo.h2 = h2;
		
		for(int i=0;i<signPoints.size();i++)
		{
			px = signPoints[i].x - mainPointX;
			double X = d*px*cos(fai2)/f/cos(fai2-theta);
			//(X,d)
		}
		
		
		f = K.at<double>(0,0);
//		double m = (signPointsBeforeStep[1].x - mainPointX)*cos(fai2s)*cos(fai2-theta);
//		double n = (signPoints[1].x - mainPointX)*cos(fai2)*cos(fai2s-theta);
		double m = (signPointsBeforeStep[1].x - signPointsBeforeStep[2].x)*cos(fai2s)*cos(fai2-theta);
		double n = (signPoints[1].x-signPoints[2].x)*cos(fai2)*cos(fai2s-theta);
		d = step*m/(n-m);
		for(int i=0;i<signPoints.size();i++)
		{
			px = signPoints[i].x - mainPointX;
			double X = d*px*cos(fai2)/f/cos(fai2-theta);
			//(X,d)
		}
		
		return reSignRefInfo;
	}
	
	TiPole::TiPole()
	{}
	
	cv::Point2d TiPole::getPointXY(const double step, const cv::Point2d polePoint, const cv::Point2d polePointBeforeStep, const cv::Point2d crossPoint, const cv::Mat K)
	{
		double u = polePoint.x;
		double cu = K.at<double>(0, 2);
		double us = polePointBeforeStep.x;
		double fx = K.at<double>(0, 0);
		double fy = K.at<double>(1, 1);
		double tantheta = (K.at<double>(1, 2) - crossPoint.y)/fy;
		double costheta = 1.0/sqrt(1.0+tantheta*tantheta);
		double X = (u-cu)*(us-cu)*step*costheta/fx/(u-us);
		double d = (us-cu)*step/(u-us);
		return cv::Point2d(X,d);
	}
	
}
