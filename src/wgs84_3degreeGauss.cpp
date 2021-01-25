#include "wgs84_3degreeGauss.h"
/*
 * x:longitude,116
 * y:latitude,40
 * X: gaussY
 * Y: gaussX
 *
 */
//int proj4_convert_WGS84_3degree_coordinate(double x, double y, double& X, double& Y)
//{
//    projPJ pj_merc, pj_latlong;
//    if (!(pj_merc = pj_init_plus("+proj=longlat +ellps=WGS84  +datum=WGS84 +no_defs")))
//        exit(1);
//    if (!(pj_latlong = pj_init_plus("+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=39500000 +y_0=0 +ellps=GRS80 +units=m +no_defs")))
//        exit(1);
//    x = x* DEG_TO_RAD;
//    y = y * DEG_TO_RAD;
//    pj_transform(pj_merc, pj_latlong, 1, 1, &x, &y, NULL);
//    std::cout.precision(12);
//    X = x - 39000000;
//    Y = y;
//    pj_free(pj_merc);
//    pj_free(pj_latlong);
//    return 0; 
//}

//bool conv84ToGauss(double& oX, double& oY, double& oZ, const double iLontitude,const double iLatitude,const double iAltitude)
//{
//    double X,Y,x,y;
//    X = iLontitude;
//    Y = iLatitude;
//    proj4_convert_WGS84_3degree_coordinate(X,Y,y,x);
//    oX = x;
//    oY = y;
//    oZ = iAltitude;
//    return true;

//}

//bool convGaussTo84(double& oLontitude, double& oLatitude, double& oAltitude, const double oX, const double oY, const double oZ)
//{
//    double X,Y;
//    X = oX;
//    Y = oY;
//    proj4_convert_3degree_WGS84_coordinate(X,Y);
//    oLontitude = Y;
//    oLatitude = X;
//    oAltitude = oZ;
//    return true;

//}



//int proj4_convert_3degree_WGS84_coordinate(double &X, double &Y)
//{
// 
//	X += 39000000;
//	projPJ pj_merc, pj_latlong;

//	if (!(pj_merc = pj_init_plus("+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=39500000 +y_0=0 +ellps=GRS80 +units=m +no_defs")))
//		exit(1);
//	if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84 +datum=WGS84 +no_defs")))
//		exit(1);

//	pj_transform(pj_merc, pj_latlong, 1, 1, &X, &Y, NULL);
//	X = X / DEG_TO_RAD;
//	Y = Y / DEG_TO_RAD;
//	
//	pj_free(pj_merc);
//	pj_free(pj_latlong);
//	return 0;
//}



//bool convertLOCATION2InsCPTData(InsCPTData& oInsCPTData, LOCATION iLocation)
//{
//    double X,Y;
//    X = iLocation.x;
//    Y = iLocation.y;
//    proj4_convert_3degree_WGS84_coordinate(X,Y);
//    oInsCPTData.lontitude = int64_t(X*100000000.0);
//    oInsCPTData.latitude = int64_t(Y*100000000.0);
//    oInsCPTData.height = iLocation.z;
//    oInsCPTData.heading = iLocation.theta;
//    oInsCPTData.pitch = iLocation.gamma;
//    oInsCPTData.roll = iLocation.psi;

//    return true;

//}


//bool convertInsCPTData2LOCATION(LOCATION& oLocation, InsCPTData iInsCPTData)
//{
//    double X,Y,x,y;
//    X = double(iInsCPTData.lontitude)/100000000.0;
//    Y = double(iInsCPTData.latitude)/100000000.0;
//    proj4_convert_WGS84_3degree_coordinate(X,Y,y,x);
//    oLocation.x = x;
//    oLocation.y = y;
//    oLocation.z = iInsCPTData.height;
//    oLocation.theta = iInsCPTData.heading;
//    oLocation.gamma = iInsCPTData.pitch;
//    oLocation.psi = iInsCPTData.roll;
//    oLocation.confidence = 1.0;

//    return true;

//}

//bool convertLOCATION842LOCATIONGauss(LOCATION& oLocation, LOCATION iLocation)
//{
//    double X,Y,x,y;
//    X = iLocation.x;
//    Y = iLocation.y;
//    proj4_convert_WGS84_3degree_coordinate(X,Y,y,x);
//    oLocation.x = x;
//    oLocation.y = y;
//    oLocation.z = iLocation.z;
//    oLocation.theta = iLocation.theta;
//    oLocation.gamma = iLocation.gamma;
//    oLocation.psi = iLocation.psi;
//    oLocation.confidence = iLocation.confidence;
//    return true;

//}
//bool convertLOCATIONGauss2LOCATION84(LOCATION& oLocation, LOCATION iLocation)
//{
//    double X,Y;
//    X = iLocation.x;
//    Y = iLocation.y;
//    proj4_convert_3degree_WGS84_coordinate(X,Y);
//    oLocation.x = Y;
//    oLocation.y = X;
//    oLocation.z = iLocation.z;
//    oLocation.theta = iLocation.theta;
//    oLocation.gamma = iLocation.gamma;
//    oLocation.psi = iLocation.psi;
//    oLocation.confidence = iLocation.confidence;
//    return true;
//}

//bool convertLOCATION84ToLOCATIONEnu(LOCATION& oLocation, LOCATION iLocation)
//{

//    oLocation = iLocation;

//    double lon  = iLocation.x;
//    double lat  = iLocation.y;
//    double alti = iLocation.z;
//    double oX,oY,oZ;
//    conv84ToEnu(oX, oY, oZ, lon, lat, alti);

//    oLocation.x = oX;
//    oLocation.y = oY;
//    oLocation.z = oZ;

//    return true;
//}
bool convertLOCATIONEnuToLOCATION84(LOCATION& oLocation, LOCATION iLocation)
{
    oLocation = iLocation;

    double iX = iLocation.x;
    double iY = iLocation.y;
    double iZ = iLocation.z;
    double oLon, oLat, oAlti;

    convEnuTo84(oLon, oLat, oAlti, iX, iY, iZ);

    oLocation.x = oLon;
    oLocation.y = oLat;
    oLocation.z = oAlti;

    return true;

}


/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double dot(const double *a, const double *b, int n)
{
    double c=0.0;

    while (--n>=0) c+=a[n]*b[n];
    return c;
}

//matmul("TN",3,1,3,1.0,E,e,0.0,r);
/* multiply matrix -----------------------------------------------------------*/
void matmul(const char *tr, int n, int k, int m, double alpha,
            const double *A, const double *B, double beta, double *C)
{
    double d;
    int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

    for (i = 0; i < n; i++)
        for (j = 0; j < k; j++)
        {
            d = 0.0;
            switch (f)
            {
                case 1: for (x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m]; break;
                case 2: for (x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k]; break;
                case 3: for (x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m]; break;
                case 4: for (x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k]; break;
            }
            if (beta == 0.0)
                C[i + j * n] = alpha * d;
            else
                C[i + j * n] = alpha * d + beta * C[i + j * n];
        }
}


/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height   经纬度--->地心地固（x,y,z）
*-----------------------------------------------------------------------------*/
void pos2ecef(const double *pos, double *r)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sin_l = sin(pos[1]), cos_l = cos(pos[1]);
    //double sinp,cosp,sin_l,cosl;
    //double tmp;
    double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp*sinp);

    //tmp = pos[0];
    //sinp = sin(tmp);

    r[0] = (v + pos[2])*cosp*cos_l;
    r[1] = (v + pos[2])*cosp*sin_l;
    r[2] = (v*(1.0 - e2) + pos[2])*sinp;
}
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)  经纬度--->3*3矩阵
*-----------------------------------------------------------------------------*/
void xyz2enu(const double *pos, double *E)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sin_l = sin(pos[1]), cos_l = cos(pos[1]);
    //double sinp,cosp,sinl,cosl;

    E[0] = -sin_l;      E[3] = cos_l;       E[6] = 0.0;
    E[1] = -sinp * cos_l; E[4] = -sinp * sin_l; E[7] = cosp;
    E[2] = cosp * cos_l;  E[5] = cosp * sin_l;  E[8] = sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none  经纬度+地心地固--->enu
*-----------------------------------------------------------------------------*/
/// 基准GPS点, 第二个地心地固坐标 - 基准地心地固, 第二个点的地心地固坐标
void ecef2enu(const double *pos, const double *r, double *e)
{
    double E[9];

    xyz2enu(pos, E);
    matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}


/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e        I   vector in local tangental coordinate {e,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none  经纬度+enu--->地心地固
*-----------------------------------------------------------------------------*/
void enu2ecef(const double *pos, const double *e, double *r)
{
    double E[9];

    xyz2enu(pos,E);
    matmul("TN",3,1,3,1.0,E,e,0.0,r);
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height   地心地固--->经纬度
*-----------------------------------------------------------------------------*/
void ecef2pos(const double *r, double *pos)
{
    double e2=FE_WGS84*(2.0-FE_WGS84);
    double r2=dot(r,r,2);
    double z,zk,v=RE_WGS84,sinp;

    for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) {
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
        z=r[2]+v*e2*sinp;
    }
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?PI/2.0:-PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
}

bool conv84ToEnu(double& oX, double& oY, double& oZ, const double iLontitude,const double iLatitude,const double iAltitude)
{
    ///demo1:根据两个GPS点，以第一个点为基准，计算第二个点的enu
    double oll[3]={40.07173118/180*PI,116.24562474/180*PI, 34.96410000};
    double oxyz[3];
    pos2ecef(oll,oxyz);

    double ll1[3]={iLatitude/180*PI, iLontitude/180*PI, iAltitude };
    double r1[3];
    pos2ecef(ll1,r1);


    double  enu[3];
    double rerrror[3]={r1[0]-oxyz[0],r1[1]-oxyz[1],r1[2]-oxyz[2]};
    ecef2enu(oll,rerrror, enu );

    oX = enu[0];
    oY = enu[1];
    oZ = enu[2];

    return true;
}

bool convEnuTo84(double& oLontitude, double& oLatitude, double& oAltitude, const double oX, const double oY, const double oZ)
{

    ///demo1:根据两个GPS点，以第一个点为基准，计算第二个点的enu
    double oll[3]={40.07173118/180*PI,116.24562474/180*PI, 34.96410000};
    double oxyz[3];
    pos2ecef(oll,oxyz);

    ///demo2:根据第二个点的enu,计算第二个点的经纬值
    double e[3]={oX,oY,oZ};
    double r[3];
    enu2ecef(oll,e,r);
    double newecef[3] = {r[0] + oxyz[0],r[1] + oxyz[1],r[2] + oxyz[2]};
    double newpos[3];
    ecef2pos(newecef ,newpos);

    oLontitude = newpos[1]*180/PI;
    oLatitude  = newpos[0]*180/PI;
    oAltitude  = newpos[2];

    return true;
}
