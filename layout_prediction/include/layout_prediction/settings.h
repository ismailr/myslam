#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include <iostream>
#include <string>
#include <libconfig.h++>

using namespace std;
using namespace libconfig;

namespace MYSLAM {
    extern int SIMULATION;
    extern int SLAM_ONLINE;
    extern string PCL_FRAME;
    extern int WALL_DETECTOR_METHOD;
    extern int WALL_DETECTOR_CLOUD_ROW;
}
#endif
