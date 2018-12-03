#include "myslam_system/pose.h"
#include "myslam_system/helpers.h"

namespace MYSLAM {
    Pose::Pose() : _active (false) { 
        _id = Generator::id++;
        _pose.setZero();
    };

    Pose3::Pose3() : _active (false) { 
        _id = Generator::id++;
        _pose.setIdentity();
    };

    PoseVertex::PoseVertex(){};

    Pose3Vertex::Pose3Vertex(){};
}
