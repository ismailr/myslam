#ifndef _MEASUREMENT_H_
#define _MEASUREMENT_H_

#include <memory>
#include <g2o/types/slam2d/edge_se2.h>

namespace MYSLAM {
    class PoseMeasurement : public EdgeSE2 
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        PoseMeasurement();

        void computeError()
        {
            const PoseVertex* v1 = static_cast<const PoseVertex*>(_vertices[0]);
            const PoseVertex* v2 = static_cast<const PoseVertex*>(_vertices[1]);
            Eigen::Vector3d prediction = (v1->estimate().inverse()*v2->estimate()).toVector();
            Eigen::Vector3d measurement = _measurement.toVector();
            _error = (measurement - prediction);
        }
    };

}
#endif
