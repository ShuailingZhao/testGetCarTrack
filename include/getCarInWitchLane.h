#ifndef GETCARINWITCHLANE_H
#define GETCARINWITCHLANE_H
struct CARPOSINFO{
    int laneInd;
    double offSetXRefLeftLane;
};

//enum INFO_CODE {NO_RIGHT_LANE, NO_LEFT_LANE, NO_ID, NO_DIST};
CARPOSINFO getLaneIndAndOffSet(const int x, const int y, const cv::Mat label);
#endif
