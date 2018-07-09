#include "myslam_system/object.h"
#include "myslam_system/helpers.h"

namespace MYSLAM {
    Object::Object() : _active (false) {
       _id = Generator::id++;
    }

    ObjectVertex::ObjectVertex() {
    }

}
