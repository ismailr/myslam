#include "slam_interface.h"

namespace MYSLAM {
    SlamInterface::SlamInterface(SparseOptimizerOnline* optimizer):g2o::G2oSlamInterface(optimizer) {};
}
