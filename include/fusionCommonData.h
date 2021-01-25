#ifndef FUSIONCOMMONDATA_H
#define FUSIONCOMMONDATA_H

//#include <opencv2/opencv.hpp>
#include <iostream>
#include  <memory>
#include <vector>
#include <Eigen/Dense>
#include "sophus/so3.h"
#include "sophus/se3.h"



typedef Eigen::Matrix<double,6,1> Vector6d;

/*
 *自车位置数据结构
 */
typedef struct LOCATION{
    uint64_t timestamp;
    bool isFusion;
    double x;
    double y;
    double z;
    double theta;
    double gamma;
    double psi;
    double confidence;
    Eigen::Vector2d hdmap_bias;
} LOCATION;

typedef struct {
    double diffx;
    double diffy;
    double diffz;
    double diffgamma;
    double diffpsi;
    double difftheta;
} LOCATION_CALIBRATION;

/*
 *车道数据结构
 */
typedef struct LANE{
    double a0;//车道方程系数 y=a0+a1x^1+a2x^2+a3x^3
    double a1;//车道方程系数 y=a0+a1x^1+a2x^2+a3x^3
    double a2;//车道方程系数 y=a0+a1x^1+a2x^2+a3x^3
    double a3;//车道方程系数 y=a0+a1x^1+a2x^2+a3x^3
    double laneRange;//车道有效范围
    double laneBackwardRange;
    unsigned int laneId;//车道id
    int laneType;//车道类型1虚线,2实线,0未知，10000混合
    unsigned char laneColor;//车道颜色1黄，2白，0未知
    int LaneConf;//车道置信度
    int n_pts_fitting;//zhao
} LANE;

/*
 *车道配对信息结构
 *eyeQ3会输出4条车道线的信息，本结构表示这4条线分别对应到的hdmap上的车道为何
 *若某条车道线无法与hdmap上的车道对应，则输出 -1
*/

typedef struct LANE_PAIR_INFO{
    int L; // eyeQ3输出的左侧第一条车道，其对应的hdmap车道id
    int R; // eyeQ3输出的右侧第一条车道，其对应的hdmap车道id
    int LL; // eyeQ3左侧第二条车道对应的hdmap车道id
    int RR; // eyeQ3右侧第二条车道对应的hdmap车道id
} LANE_PAIR_INFO;


/*
 *traffic sign
 */
typedef struct SIGNBOARD{
    Eigen::Vector3d boardCenter;//牌子中心点
    double width;
    double height;
    uint8_t shape;
    uint16_t type;
} SIGNBOARD;

/*
 *道路数据结构 hdmap
 */
typedef struct LANE_ROAD_INFO{
    LOCATION pos; //自车位置高斯坐标
    std::vector<LANE> hdmapLanes; //车道线
    std::vector<Eigen::Vector3d> pts;//车道线切点
    std::vector<SIGNBOARD> signBoards;//牌子中心点
    double road_heading;
    double road_slope; // 除10000.0
    double road_superelevation;
    double road_curvature; //除1000000.0
} LANE_ROAD_INFO;



typedef struct EYEQ3_HDMAP_LANE_ROAD{
    LANE_PAIR_INFO lanePairInfo; //自eyeQ3四条车道线对应到的hdmap上的车道线之id
    std::vector<LANE> eyeQ3Lanes;//eyeQ3车道
    std::vector<LANE> hdmapLanes;//hdmap车道
    std::vector<SIGNBOARD> signBoards;//牌子中心点
} EYEQ3_HDMAP_LANE_ROAD;



typedef struct EYEQ3DATA{
    std::vector<LANE> eyeQ3Data;
    //uint64_t timestamp;
} EYEQ3DATA;

//struct EYEQ3DATA{
//    std::vector<LANE>& eyeQ3Data;
//};

/*
 *eyeQ3和道路数据结构
 */
typedef struct EYEQ3_LANE_ROAD_INFO{
    EYEQ3DATA eyeQ3Data;
    LANE_ROAD_INFO laneRoadInfo;
} EYEQ3_LANE_ROAD_INFO;


typedef struct OffSetX
{
    uint64_t timestamp;
    double offSetX;
    unsigned char lrm;
} OFFSETX;

typedef struct _VIS_LANE_NEAR_LEFT
{
    uint64_t timestamp;
    double VIS_LANE_LEFT_A0;
    double VIS_LANE_LEFT_A2;
    double VIS_LANE_LEFT_A1;
    double VIS_LANE_LEFT_A3;
    unsigned char VIS_LANE_LEFT_MAKER_COLOR;
    double VIS_LANE_LEFT_RANNGE;
    unsigned int ID;
} EYEQ3LANE;

typedef struct _VIS_LANE_INFORMATION
{
    unsigned char VIS_LANE_LEFT_INDIVID_TYPE;
    unsigned char VIS_LANE_LEFT_PARALL_TYPE;
    unsigned char VIS_LANE_RIGHT_INDIVID_TYPE;
    unsigned char VIS_LANE_RIGHT_PARALL_TYPE;
    unsigned char VIS_LANE_LEFT_PARALL_DIMONCONF;
    unsigned char VIS_LANE_LEFT_PARALL_LKACONF;
    unsigned char VIS_LANE_LEFT_PARALL_TJACONF;
    unsigned char VIS_LANE_LEFT_INDIVID_DIMONCONF;
    unsigned char VIS_LANE_LEFT_INDIVID_LKACONF;
    unsigned char VIS_LANE_LEFT_INDIVID_TJACONF;
    unsigned char VIS_LANE_RIGHT_PARALL_DIMONCONF;
    unsigned char VIS_LANE_RIGHT_PARALL_LKACONF;
    unsigned char VIS_LANE_RIGHT_PARALL_TJACONF;
    unsigned char VIS_LANE_RIGHT_INDIVID_DIMONCONF;
    unsigned char VIS_LANE_RIGHT_INDIVID_LKACONF;
    unsigned char VIS_LANE_RIGHT_INDIVID_TJACONF;
    unsigned char VIS_LANE_LEFT_NEIGHBOR_TYPE;
    unsigned char VIS_LANE_RIGHT_NEIGHBOR_TYPE;
    unsigned char VIS_LANE_LEFT_NEIGHBOR_LKACONF;
    unsigned char VIS_LANE_RIGHT_NEIGHBOR_LKACONF;
    unsigned char VIS_LANE_LANE_CHANGE;
    //unsigned char VIS_LANE_AMBIGUOUS_LANE_LEFT;
    //unsigned char VIS_LANE_AMBIGUOUS_LANE_RIGHT;
    //unsigned char VIS_ROAD_ISTUNNEL_ENTRYEXIT;
    //unsigned char VIS_LANE_PARALL_PROB
    unsigned int ID;
    uint64_t timestamp;
} EYEQ3LANEINFO;

typedef struct EYEQ3ROWDATA
{
    std::vector<EYEQ3LANEINFO> vEyeQ3LaneInfo;
    std::vector<EYEQ3LANE> vEyeQ3LLLane;
    std::vector<EYEQ3LANE> vEyeQ3LLane;
    std::vector<EYEQ3LANE> vEyeQ3RLane;
    std::vector<EYEQ3LANE> vEyeQ3RRLane;
    std::vector<OffSetX> vOffSetXData;

} EYEQ3ROWDATA;

typedef struct wheelDRPos
{
    double x;
    double y;
    double theta;
} WHEELDRPOS;

struct VehicleStatusData
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
};

//Ins Data (CPT)
struct InsCPTData
{

    int64_t  timestamp;

    /// milliseconds
    int64_t  latitude;

    /// 10^8
    int64_t  lontitude;

    /// 10^8
    double height;

    /// m
    double speed;

    /// m/s
    double roll;

    /// degree
    double pitch;

    /// degree
    double heading;

    /// degree
    double xVelocity;

    double yVelocity;

    double zVelocity;

    double xAngularRate;

    double yAngularRate;

    double zAngularRate;

    double xAcceleration;

    double yAcceleration;

    double zAcceleration;
};


typedef struct FusionData
{
    InsCPTData rawRTK;
    InsCPTData rawGps;
    VehicleStatusData vehiState;
    wheelDRPos wheelDR;
    LOCATION vPos;
    LOCATION fusiPos;
    EYEQ3DATA eyeQ3Data;
    LANE_ROAD_INFO laneRoadInfo;
    LOCATION_CALIBRATION locationCal;

} FUSIONDATA;

#endif

