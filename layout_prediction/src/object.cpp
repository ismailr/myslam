#include "layout_prediction/object.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    Object::Object() {
       _id = Generator::id++;
    }

    ObjectVertex::ObjectVertex() {
    }

}
