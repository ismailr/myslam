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
    extern double A5;
    extern double A6;
    extern double B1;
    extern double B2;
    extern double B3;

    void loadConfFile(const char* fileconfig);
}
#endif
