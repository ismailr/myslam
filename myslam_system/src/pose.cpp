#include "myslam_system/pose.h"
#include "myslam_system/helpers.h"

namespace MYSLAM {
    Pose::Pose() : _active (false) { 
        _id = Generator::id++;
        _pose.setZero();
    };

    PoseVertex::PoseVertex(){};
}
