#include "layout_prediction/object.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    Object::Object() {}

    ObjectVertex::ObjectVertex() {
       _id = Generator::id++;
    }

}
