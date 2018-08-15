#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include <iostream>
#include <string>
#include <libconfig.h++>

using namespace std;
using namespace libconfig;

namespace MYSLAM {
    extern double TPPF1;
    extern double TPPF2;
    extern double TPPF3;
    extern double TPPF4;

    void loadConfFile(const char* fileconfig);
}
#endif
