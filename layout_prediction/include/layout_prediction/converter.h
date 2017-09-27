#ifndef _CONVERTER_H_
#define _CONVERTER_H_

#include <nav_msgs/Odometry.h>
#include "layout_prediction/se2.h"

class Converter
{
    public:
        Converter(){};
        void odomToSE2 (nav_msgs::OdometryConstPtr& odom, SE2& t);
};

#endif
