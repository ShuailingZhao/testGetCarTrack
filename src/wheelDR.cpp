#include "wheelDR.h"

namespace dr
{
	WheelDR::WheelDR()
	{}
	
	double WheelDR::getDisWithWheelDR(const std::vector<VehicleStatusData> ivVehicleStatusData)
	{
		WheelDRPos oWDRPos;
		WheelDRPos iVDRPos;
		iVDRPos.x = 0.0;
		iVDRPos.y = 0.0;
		iVDRPos.theta = 0.0;//车头方向相对于坐标系的方向角，逆时针为正
		getPosRefWheelDR(oWDRPos, iVDRPos, ivVehicleStatusData);
		double distance = sqrt((oWDRPos.x-iVDRPos.x)*(oWDRPos.x-iVDRPos.x)+(oWDRPos.y-iVDRPos.y)*(oWDRPos.y-iVDRPos.y));
		return distance;	
	}
	
	bool WheelDR::getPosRefWheelDR(struct WheelDRPos& oWDRPos, const struct WheelDRPos iVDRPos, const std::vector<VehicleStatusData> ivVehicleStatusData)
	{

	    if(ivVehicleStatusData.size()<2)
	    {
		return false;
	    }

	    std::vector<VehicleStatusData> vVehicleStatusData;
	    vVehicleStatusData.assign(ivVehicleStatusData.begin(),ivVehicleStatusData.end());
	    struct WheelDRPos iVdrPos;
	    iVdrPos.x       = iVDRPos.x;
	    iVdrPos.y       = iVDRPos.y;
	    iVdrPos.theta   = iVDRPos.theta;

	//    iVdrPos.x       = -1.0*(iVDRPos.x);
	//    iVdrPos.y       = iVDRPos.y;
	//    iVdrPos.theta   = 180.0 - iVDRPos.theta;


	    double ddtheta;
	    //for(int64_t vehicleInd=(int64_t)(std::stol(starFileName.substr(0,starFileName.size()-4))) ; vehicleInd<(int64_t)(std::stol(endFileName.substr(0,endFileName.size()-4))); )
	    for(int indWheel = 0; indWheel<ivVehicleStatusData.size()-1; indWheel++)
	    {

		//get distance with vehicle
		struct WheelDRPos owDRPos;
		double wheeltheta =  double(vVehicleStatusData[1+indWheel].wheelAngle - vVehicleStatusData[0+indWheel].wheelAngle);
		getPosRefWheelDRStep(ddtheta, iVdrPos, vVehicleStatusData[0+indWheel], (vVehicleStatusData[1+indWheel].timestamp-vVehicleStatusData[0+indWheel].timestamp), owDRPos, wheeltheta);

		iVdrPos = owDRPos;//update
	    }

	    oWDRPos.x       = iVdrPos.x;
	    oWDRPos.y       = iVdrPos.y;
	    oWDRPos.theta   = iVdrPos.theta;

	//    oWDRPos.x       = -1.0*iVdrPos.x;//the difference of the gauss coordinate and cartesian coordinate
	//    oWDRPos.y       = iVdrPos.y;
	//    oWDRPos.theta   = -1.0*(iVdrPos.theta - 180.0);

	    return true;
	}
	
	bool WheelDR::getPosRefWheelDRStep(double& ddtheta, struct WheelDRPos iVDRPos, struct VehicleStatusData iVStatus, int64_t ideltaTime, struct WheelDRPos& owDRPos, double wheeltheta)
	{
	    double x0,y0,theta0, x1,y1,theta1;
	    double dx,dy,dtheta;
	    double r, l, e, L, vr,vl,vrb,vlb,vcar;
	    double deltaTime = ideltaTime/1000.0;
	    double pi = 3.14159265358979;
	    double dthetaCan;

	    x0        = iVDRPos.x;
	    y0        = iVDRPos.y;
	    theta0    = iVDRPos.theta*pi/180.0;//iVDRPos.theta*pi/180;

	    vl        = iVStatus.leftFrontSpeed/10.0;
	    vr        = iVStatus.rightFrontSpeed/10.0;
	    vlb       = iVStatus.leftRearSpeed/10.0;
	    vrb       = iVStatus.rightRearSpeed/10.0;
	    vcar      = iVStatus.vehicleSpeed/10.0;
	    dthetaCan = iVStatus.wheelAngle/10.0/16.0;


	    double tempTheta=0.0;

	    l       = 1.655;// problem
	    L       = 2.86;
	    e       = l/2;
	    if(fabs(vlb-vrb)<0.2)//前轮0.3 后轮0.2
	    {
		r       = 1e63;
		dtheta  = 0.0;
		dy      = 0.0;
		dx      = vcar*deltaTime;

	    }else
	    {
		r       = l*(vrb+vlb)/(vrb-vlb)/2.0;
		dtheta  = (vrb-vlb)*deltaTime/l;
		dy      = r*(1-cos(dtheta));
		dx      = r*sin(dtheta);

	    }

	    x1 = x0 + dx*cos(theta0) - dy*sin(theta0);
	    y1 = y0 + dx*sin(theta0) + dy*cos(theta0);
	    theta1 = theta0 + dtheta;

	    owDRPos.x       = x1;
	    owDRPos.y       = y1;
	    owDRPos.theta   = theta1*180.0/pi;

	    ddtheta = dtheta;

	    return true;
	}
	
	bool WheelDR::isFindTimeStampRow(int64_t timeStamp, std::vector<VehicleStatusData> vVehicleStatusData)
	{
		if(2 != vVehicleStatusData.size())
			return false;
		if((timeStamp-vVehicleStatusData[0].timestamp+10>=0)&&(abs(vVehicleStatusData[0].timestamp-timeStamp) <= abs(vVehicleStatusData[1].timestamp-timeStamp)))
			return true;
	}
}


