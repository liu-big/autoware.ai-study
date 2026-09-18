#include <ros/ros.h>
#include <iostream>
#include "lidar_obstacle_detection/safe_range_core.h"

using namespace std;

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "safe_range_ctrl");
    SafeRangeNs::SafeRange safe_range;
    safe_range.MainLoop();
    return 0;
}
