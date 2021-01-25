#include "getXYawPitch.h"
void getCrossPoint(std::vector<cv::Point2d>lines, cv::Point2d& cross_Pt)
{
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
    Point2d tmp_pt;
    if(abs(0.0-D)<0.00001)
    {
        tmp_pt.x=0.0;
        tmp_pt.y=0.0;
    }

    double x = (b1*c3 - b3*c1) / D;
    double y = (a3*c1 - a1*c3) / D;

    tmp_pt.x = x;
    tmp_pt.y = y;
    cross_Pt = tmp_pt;
}

void transformToBirdViewLine(std::vector<cv::Point2d>& birdViewLines, std::vector<cv::Point2d> lines, cv::Mat m)
{
    for(int i=0; i<lines.size();i++)
    {
        Mat points(3,1,CV_64F);
        points.at<double>(0,0) = lines[i].x;
        points.at<double>(1,0) = lines[i].y;
        points.at<double>(2,0) = 1;
        Mat dis = m*points;

        dis.at<double>(0,0) = dis.at<double>(0,0)/dis.at<double>(2,0);
        dis.at<double>(1,0) = dis.at<double>(1,0)/dis.at<double>(2,0);
        dis.at<double>(2,0) = dis.at<double>(2,0)/dis.at<double>(2,0);

        cv::Point2d birdViewPoint;
        birdViewPoint.x=dis.at<double>(0,0);
        birdViewPoint.y=dis.at<double>(1,0);

        birdViewLines.push_back(birdViewPoint);

    }

}
bool getXYawPitch(double& offx, double& yaw, double& pitch, std::vector<cv::Point2d> lines, cv::Mat intrinsicMatrix, cv::Mat distCoeffs, cv::Mat H, Size imageSize,cv::Point2d vanishPoint, double condis)
{
    if(4 != lines.size())
        return false;

    double constDis=0;//Has problem
    if(!abs(0 - condis)<0.0001)
        constDis = condis;
    else
    {
        constDis = 0;

    }

    Size birdViewSize(220,320);

    cv::Point2d vP(0,0);

    //Get the vanishing point
    if(0 == vanishPoint.x && 0 == vanishPoint.y)
        getCrossPoint(lines, vP);
    else
        vP = vanishPoint;//vanishing point

    std::vector<cv::Point2d> midLine;
    std::vector<cv::Point2d> birdViewLines;
    cv::Point2d topPoint;
    cv::Point2d bottomPoint;

    cout << "Line:" <<endl;

    cout << lines[0] <<endl;
    cout << lines[1] <<endl;
    cout << lines[2] <<endl;
    cout << lines[3] <<endl;


    //Tranform the lines to birdViewLines

    //cv::undistortPoints //If the lines is in the original image, they should be undistorted first.
    transformToBirdViewLine(birdViewLines, lines, H);

    cout << "birdViewLine:" <<endl;

    cout << birdViewLines[0] <<endl;
    cout << birdViewLines[1] <<endl;
    cout << birdViewLines[2] <<endl;
    cout << birdViewLines[3] <<endl;


    //Get the middle line
    bottomPoint.x = (birdViewLines[0].x + birdViewLines[2].x)/2.0;
    bottomPoint.y = (birdViewLines[0].y + birdViewLines[2].y)/2.0;
    topPoint.x = (birdViewLines[1].x + birdViewLines[3].x)/2.0;
    topPoint.y = (birdViewLines[1].y + birdViewLines[3].y)/2.0;
    midLine.push_back(bottomPoint);
    midLine.push_back(topPoint);


    //Get the yaw, and will be better if birdview image is larger.
    double rTri0 = midLine[1].x - midLine[0].x;
    double rTri1 = midLine[1].y - midLine[0].y;
    if(abs(0-rTri0)<0.0001)
        yaw=0.0;
    else
    {
        yaw = -1*atan(rTri0/rTri1)*180.0/3.1415926;//The value will be negative if look left and positive if right.
    }



    //Get the new camera matrix
    double alpha=0;//0 kuo, 1 suo
    Size rectificationSize = imageSize;
    Rect validPixROI;
    cv::Mat P;
    P= getOptimalNewCameraMatrix(intrinsicMatrix,distCoeffs,
                                 imageSize,
                                 alpha,//0 huo, 1 suo
                                 rectificationSize,//undistorted image size
                                 &validPixROI//undistorted image rectangle in source image
                                 );//new camera matirx

    //Get the pitch,the value will be better if P is better.
    double halfH = P.at<double>(1,2);
    double fy= P.at<double>(1,1);
    pitch = atan((halfH - vP.y)/fy)*180.0/3.1415926;//The value will be positive if camera looks down and negative looks up.



    //Get the offset, that wil be negative in the left size, positive in the right side.
    if(abs(0.0 - yaw)<0.0001)
    {
        offx = midLine[0].x/100.0;//One pixel 10cm or cm to m
    }else if(abs(constDis-0.0)<0.0001)
    {
        double bottomx, bottomy;
        bottomy = 0.0;
        cout << "----------" <<endl;
        cout << midLine[0]<<endl;
        cout << midLine[1]<<endl;


        bottomx = midLine[1].x - ((midLine[1].x-midLine[0].x)/(midLine[1].y-midLine[0].y))*(midLine[1].y - bottomy);
        offx = bottomx/100.0;//One pixel 10cm or cm to m
        cout << offx<<endl;

    }else
    {
        std::vector<cv::Point2d> CrossLines;
        CrossLines.push_back(midLine[0]);
        CrossLines.push_back(midLine[1]);
        CrossLines.push_back(cv::Point2d(0.0, 0.0));
        CrossLines.push_back(cv::Point2d(0.0, 100.0));

        cv::Point2d cross_Pt;
        getCrossPoint(CrossLines, cross_Pt);
        double dis = cross_Pt.y + constDis;
        if(dis<0.0)
            return false;
        offx = dis * sin(yaw*3.1415926/180.0);
    }

    return true;
}
