#include <iostream>
#include "wheelDR.h"


std::vector<dr::VehicleStatusData> getvechcleStatusData()
{
	std::vector<dr::VehicleStatusData> ivVehicleStatusData;
	dr::VehicleStatusData vehiSta;
	vehiSta.timestamp = 1557963817014;
	vehiSta.wheelAngle = -8;
	vehiSta.Acc0 = 0;
	vehiSta.Acc1 = 0;
	vehiSta.Acc2 = 0;
	vehiSta.brakePedal = 1;
	vehiSta.leftFrontSpeed = 140;
	vehiSta.rightFrontSpeed = 138;
	vehiSta.leftRearSpeed = 140;
	vehiSta.rightRearSpeed = 139;
	vehiSta.vehicleSpeed = 140;
	ivVehicleStatusData.push_back(vehiSta);


	vehiSta.timestamp = 1557963817034;
	vehiSta.wheelAngle = -6;
	vehiSta.Acc0 = 0;
	vehiSta.Acc1 = 0;
	vehiSta.Acc2 = 0;
	vehiSta.brakePedal = 1;
	vehiSta.leftFrontSpeed = 140;
	vehiSta.rightFrontSpeed = 138;
	vehiSta.leftRearSpeed = 139;
	vehiSta.rightRearSpeed = 139;
	vehiSta.vehicleSpeed = 139;
	ivVehicleStatusData.push_back(vehiSta);
	
	return ivVehicleStatusData;
}

int main()
{
    
    
    std::vector<dr::VehicleStatusData>  vehicleStatusData = getvechcleStatusData();
    dr::WheelDR wheelDR;
    double distance = wheelDR.getDisWithWheelDR(vehicleStatusData);
    std::cout<<"distance "<<distance<<std::endl;


    return 1;
}
