#ifndef WHEELDR_H
#define WHEELDR_H
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
//#include "v_location.h"
//#include "CSVRow.h"
//#include "ToolsBox.h"

typedef short int int16_t;

# if __WORDSIZE == 64
typedef long int int64_t;
# else
__extension__
typedef long long int int64_t;

#endif

namespace dr
{
	typedef struct VehicleStatusData
	{
	    int64_t    timestamp;

	    int16_t    wheelAngle;

	    /// wheelAngle=X*10
	    int16_t    Acc0;

	    /// Acc0=X*10, range: -5~7
	    int16_t    Acc1;

	    /// range: 0~99
	    int16_t    Acc2;

	    /// range: 0~99
	    int16_t    brakePedal;

	    int16_t    leftFrontSpeed;

	    /// wheelSpeed=X/10
	    int16_t    rightFrontSpeed;

	    int16_t    leftRearSpeed;

	    int16_t    rightRearSpeed;

	    int16_t    vehicleSpeed;
	} VehicleStatusData;
	
	typedef struct WheelDRPos
	{
	    double x;
	    double y;
	    double theta;
	} WheelDRPos;
	
	class WheelDR
	{
		public:
			WheelDR();
			double getDisWithWheelDR(const std::vector<VehicleStatusData> ivVehicleStatusData);
			bool getPosRefWheelDR(struct WheelDRPos& oWDRPos, const struct WheelDRPos iVDRPos, const std::vector<VehicleStatusData> ivVehicleStatusData);
			bool getPosRefWheelDRStep(double& ddtheta, struct WheelDRPos iVDRPos, struct VehicleStatusData iVStatus, int64_t ideltaTime, struct WheelDRPos& owDRPos, double wheeltheta);
			bool isFindTimeStampRow(int64_t timeStamp, std::vector<VehicleStatusData> vVehicleStatusData);
			
			
		private:
	};
}
	

#endif
