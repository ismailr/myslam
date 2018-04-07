#include "layout_prediction/pose.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    Pose::Pose() { 
        _id = Generator::id++;
        _pose.setZero();
    };

    PoseVertex::PoseVertex(){};
}
