#ifndef WGS84TO3DEGREEGAUSS_H
#define WGS84TO3DEGREEGAUSS_H

#include <iostream>
#include <proj_api.h>
#include <fstream>
#include <vector>
#include <iomanip>
#include <math.h>
#include <sstream>
#include "fusionCommonData.h"
//using namespace std;

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */
#define PI 3.141592657

//84 与 高斯坐标互转
//int proj4_convert_WGS84_3degree_coordinate(double x, double y, double& X, double& Y);
//int proj4_convert_3degree_WGS84_coordinate(double &X, double &Y);
//bool conv84ToGauss(double& oX, double& oY, double& oZ, const double iLontitude,const double iLatitude,const double iAltitude);
//bool convGaussTo84(double& oLontitude, double& oLatitude, double& oAltitude, const double oX, const double oY, const double oZ);

//bool convertLOCATION2InsCPTData(InsCPTData& oInsCPTData, LOCATION iLocation);
//bool convertInsCPTData2LOCATION(LOCATION& oLocation, InsCPTData iInsCPTData);
//bool convertLOCATION842LOCATIONGauss(LOCATION& oLocation, LOCATION iLocation);
//bool convertLOCATIONGauss2LOCATION84(LOCATION& oLocation, LOCATION iLocation);

//84 与东北天坐标互转
double dot(const double *a, const double *b, int n);
void matmul(const char *tr, int n, int k, int m, double alpha,
            const double *A, const double *B, double beta, double *C);
void pos2ecef(const double *pos, double *r);
void xyz2enu(const double *pos, double *E);
void ecef2enu(const double *pos, const double *r, double *e);
void enu2ecef(const double *pos, const double *e, double *r);
void ecef2pos(const double *r, double *pos);
bool conv84ToEnu(double& x, double& y, double& z, const double lontitude,const double latitude,const double altitude);
bool convEnuTo84(double& lontitude, double& latitude, double& altitude, const double x, const double y, const double z);

bool convertLOCATION84ToLOCATIONEnu(LOCATION& oLocation, LOCATION iLocation);
bool convertLOCATIONEnuToLOCATION84(LOCATION& oLocation, LOCATION iLocation);

#endif
