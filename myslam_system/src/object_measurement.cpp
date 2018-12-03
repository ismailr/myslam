#include "myslam_system/object_measurement.h"
#include "myslam_system/helpers.h"

using namespace Eigen;

namespace MYSLAM {
    ObjectMeasurement::ObjectMeasurement() : EdgeSE2() {};
    ObjectXYZMeasurement::ObjectXYZMeasurement() 
        :_active (false) {
        _id = Generator::id++;
        _cov.setIdentity();
    };

    Pose3ObjectXYZEdge::Pose3ObjectXYZEdge() : EdgeSE3PointXYZ() {};
}

