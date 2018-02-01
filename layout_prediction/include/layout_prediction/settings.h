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
    extern double DATA_ASSOCIATION_THRESHOLD;
    extern int SIM_NUMBER_OF_ITERATIONS;
    extern int TRACKER_METHOD;
    extern int PF_NUMBER_OF_PARTICLES;

    void loadConfFile(const char* fileconfig);
}
#endif
