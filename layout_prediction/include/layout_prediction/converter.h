#ifndef _CONVERTER_H_
#define _CONVERTER_H_

#include <nav_msgs/Odometry.h>
#include "se2.h"

using namespace g2o;

class Converter
{
    public:
        Converter(){};
        void odomToSE2 (nav_msgs::OdometryConstPtr& odom, SE2& t);
};

#endif
