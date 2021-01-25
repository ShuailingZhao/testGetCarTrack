#include "getAngle.h"
#include <cmath>


//定义Point2f结构体
struct Point2D
{
    double x;
    double y;
};

// 定义直线参数结构体
struct LinePara
{
    double k;
    double b;

};

// 获取直线参数
void getLinePara(double& x1, double& y1, double& x2, double& y2, LinePara & LP)
{
    int m = 0;

    // 计算分子
    m = x2 - x1;

    if (0 == m)
    {
        LP.k = 10000.0;
        LP.b = y1 - LP.k * x1;
    }
    else
    {
        LP.k = (y2 - y1) / (x2 - x1);
        LP.b = y1 - LP.k * x1;
    }


}

// 获取交点
bool getCross(double & x1, double &y1, double & x2, double & y2, double & x3, double &y3, double & x4, double & y4,  Point2D & pt)
{

    LinePara para1, para2;
    getLinePara(x1, y1, x2, y2, para1);
    getLinePara(x3, y3, x4, y4, para2);

    // 判断是否平行
    if (abs(para1.k - para2.k) > 0.5)
    {
        pt.x = (para2.b - para1.b) / (para1.k - para2.k);
        pt.y = para1.k * pt.x + para1.b;

        return true;

    }
    else
    {
        return false;
    }

}

double getAng(const double Width, const double Height, const double Fy, const double Hy,\
        const double P0x, const double P0y, const double P1x, const double P1y,\
        const double P2x, const double P2y, const double P3x, const double P3y,\
        const double P4x, const double P4y,\
        const double Heading, const double Pitch, const double Roll,\
        const double HOverGround)
{
    if(Width<=0.0 || Height<=0.0 || Fy<=0.0 || P0x<0.0 || P0y<0.0 || P1x<0.0 || P1y<0.0 || Pitch<0.0 || HOverGround<=0.0)
        return 0.0;

    double width = Width;
    double height = Height;
    double fy = Fy;
    double hy = Hy;
    double p0x = P0x;
    double p0y = P0y;
    double p1x = P1x;
    double p1y = P1y;
    double p2x = P2x;
    double p2y = P2y;
    double p3x = P3x;
    double p3y = P3y;
    double p4x = P4x;
    double p4y = P4y;
    double Central_axis0x = width/2;
    double Central_axis0y = height;
    double Central_axis1x = width/2;
    double Central_axis1y = 0;

    double heading = Heading;
    double pitch = Pitch;
    double roll = Roll;
    double hOverGround = HOverGround;

    if(!(pitch>=0 && pitch<=90))
        return 0;

    double theta = 0;
    // 获取交点
    Point2D CrossPoint;
    if(false == getCross(p0x, p0y, p1x, p1y, Central_axis0x, Central_axis0y, Central_axis1x, Central_axis1y,  CrossPoint))
        return theta;

    if(abs(p0x-width/2) < abs(p1x-width/2))
    {
        p0x = CrossPoint.x;
        p0y = CrossPoint.y;
    }else
    {
        p1x = CrossPoint.x;
        p1y = CrossPoint.y;
    }

    double pi = 3.1415926;
    double H1,H2,h2;
    double yaw;
    yaw = (90.0-pitch)*pi/180.0;
    H1 = hOverGround * tan( yaw - atan((p1y - height/2)/fy));
    H2 = hOverGround * tan( yaw - atan((p0y - height/2)/fy));
    double Y = H1 - H2;
    double m;

    if(yaw - atan((p0y-height/2)/fy) >= 90.0*pi/180.0)
        h2 = 1e8;
    else
        h2 = hOverGround/cos(yaw - atan((p0y-height/2)/fy));
    m = sqrt(fy*fy+(p0y-height/2)*(p0y-height/2));
    double X = (p1x-p0x)*h2/m;

    theta = atan(X/Y)*180/pi;
    return theta;
}
