#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include <iostream>
#include <string>
#include <libconfig.h++>

using namespace std;
using namespace libconfig;

namespace MYSLAM {
    extern int KEYFRAME_POINT;
    extern int OPTIMIZATION_POINT;
    extern double A1;
    extern double A2;
    extern double A3;
    extern double A4;
    extern double DIST_NOISE;
    extern double THETA_NOISE;

    void loadConfFile(const char* fileconfig);
}
#endif
