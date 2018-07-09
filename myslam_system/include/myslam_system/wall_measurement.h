#ifndef _WALL_MEASUREMENT_H_
#define _WALL_MEASUREMENT_H_

#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "myslam_system/wall.h"

namespace MYSLAM {
    class WallMeasurement : public EdgeSE2PointXY
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            WallMeasurement();
    };
}
#endif
