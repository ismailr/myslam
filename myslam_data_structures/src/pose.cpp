#include "myslam_data_structures/pose.h"
#include "myslam_data_structures/id_generator.h"

namespace MYSLAM {
    Pose3D::Pose3D() : _active(false) { 
        _id = Generator::id++;
        _pose.setIdentity();
    };

    Pose2D::Pose2D() { 
        _id = Generator::id++;
        _pose.setZero();
    };
}
