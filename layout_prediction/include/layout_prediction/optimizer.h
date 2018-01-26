#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include <g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.h>
#include <g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h>
#include <slam_parser/interface/parser_interface.h>

#include "layout_prediction/system.h"
#include "layout_prediction/graph.h"
#include "sparse_optimizer_incremental.h"
#include "slam_interface.h"

#include "isam/isam.h"

namespace MYSLAM {
    class Graph;
    class Optimizer
    {
        public:
            Optimizer(System&, MYSLAM::Graph&);
            Optimizer(System&, MYSLAM::Graph&, isam::Slam&);
            Optimizer(MYSLAM::Graph&); // simulation
            void localOptimize();
            void globalOptimize();
            void globalOptimizePoint();
            void incrementalOptimize();

        private:
            System *_system;
            Graph *_graph;

            SparseOptimizerIncremental *_incOptimizer;
            SlamInterface *_slamInterface;

            isam::Slam *_slam;
    };
}

#endif
