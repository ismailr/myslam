#include "myslam_data_structures/object.h"
#include "myslam_data_structures/id_generator.h"

namespace MYSLAM {
    Object3D::Object3D() : _active (false) {
       _id = Generator::id++;
       _pose.setIdentity();
    }

    Object2D::Object2D() : _active (false) {
       _id = Generator::id++;
       _pose.setZero();
    }
}
