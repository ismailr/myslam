#include "myslam_system/pose_measurement.h"
#include "myslam_system/helpers.h"

using namespace Eigen;

namespace MYSLAM {
    PoseMeasurement::PoseMeasurement() : EdgeSE2() {};
    Pose3Measurement::Pose3Measurement()
        : _active (false)
    { 
        _id = Generator::id++;
        _cov.setIdentity();
    };

    Pose3Edge::Pose3Edge():EdgeSE3() {};
}

