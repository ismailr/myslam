#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include "layout_prediction/graph.h"

namespace MYSLAM {
    class System;
    class Optimizer
    {
        public:
            Optimizer(System&, MYSLAM::Graph&);
            Optimizer(MYSLAM::Graph&); // simulation
            void localOptimize();
            void localOptimize2(Pose::Ptr pose);
            void globalOptimize();

        private:
            System *_system;
            Graph *_graph;
    };
}

#endif