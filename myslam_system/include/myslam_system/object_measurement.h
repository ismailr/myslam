#ifndef _MYSLAM_OBJECT_MEASUREMENT_H
#define _MYSLAM_OBJECT_MEASUREMENT_H

#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>

#include "myslam_system/object.h"


namespace MYSLAM {
    class ObjectMeasurement : public EdgeSE2 
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            ObjectMeasurement();
    };

    class ObjectXYZMeasurement 
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            typedef std::shared_ptr<ObjectXYZMeasurement> Ptr;

            ObjectXYZMeasurement();
            int _from;
            int _to;
            Eigen::Vector3d _measurement;
            bool _active;
            unsigned long int _id;
            Eigen::Matrix<double,3,3> _cov;
    };

    class Pose3ObjectXYZEdge : public EdgeSE3PointXYZ
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            Pose3ObjectXYZEdge();
    };
}
#endif
