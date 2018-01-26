#ifndef _POINT_H_
#define _POINT_H_

#include "layout_prediction/helpers.h"

namespace MYSLAM {
    class Point {
        public:
            typedef std::shared_ptr<Point> Ptr;
            unsigned long int _id;
            Point() : x(0.0), y(0.0) { _id = Generator::id++;};

            double x, y;
    };
}

#endif
