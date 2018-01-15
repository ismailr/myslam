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
#include "isam/pose2d.h"
#include "isam/pose2d_wall2d_factor.h"

namespace MYSLAM {
    class Graph;
    class Optimizer
    {
        public:
            Optimizer(System&, MYSLAM::Graph&);
            Optimizer(System&, MYSLAM::Graph&, isam::Slam&); // ISAM
            Optimizer(MYSLAM::Graph&); // simulation
            void localOptimize();
            void globalOptimize();
            void incrementalOptimize(int, std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >);

        private:
            System *_system;
            Graph *_graph;

            SparseOptimizerIncremental *_incOptimizer;
            SlamInterface *_slamInterface;

            /* ISAM */
            isam::Slam *_slam;
            std::vector<Pose2d_Node*> _poseNodes;
    };
}

#endif
