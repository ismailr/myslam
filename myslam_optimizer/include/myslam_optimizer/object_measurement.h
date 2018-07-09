#ifndef _MYSLAM_OBJECT_MEASUREMENT_H
#define _MYSLAM_OBJECT_MEASUREMENT_H

#include <g2o/types/slam2d/edge_se2.h>

#include "layout_prediction/object.h"


namespace MYSLAM {
    class ObjectMeasurement : public EdgeSE2 
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            ObjectMeasurement();
    };

}
#endif
