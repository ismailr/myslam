#ifndef _SPARSE_OPTIMIZER_ONLINE_H_
#define _SPARSE_OPTIMIZER_ONLINE_H_

#include "g2o/core/sparse_optimizer.h"
#include <g2o/examples/interactive_slam/g2o_interactive/graph_optimizer_sparse_online.h>

namespace MYSLAM {
    class SparseOptimizerOnline : public g2o::SparseOptimizerOnline
    {
        public:
            SparseOptimizerOnline();
            int optimize(int iterations, bool online);
            void update(double* update);
            bool initSolver(int dimension, int /*batchEveryN*/);
    };
};
#endif
