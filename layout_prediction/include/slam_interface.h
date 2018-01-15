#ifndef _SLAM_INTERFACE_H_
#define _SLAM_INTERFACE_H_

#include <g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h>
#include "sparse_optimizer_online.h"

namespace MYSLAM {
    class SlamInterface : public g2o::G2oSlamInterface
    {
        public:
            SlamInterface(SparseOptimizerOnline* optimizer);

    };
};

#endif
