#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include "layout_prediction/system.h"
#include "layout_prediction/graph.h"

namespace MYSLAM {
    class Graph;
    class Optimizer
    {
        public:
            Optimizer(System&, MYSLAM::Graph&);
            Optimizer(MYSLAM::Graph&); // simulation
            void localOptimize();
            void loopClosure();
            void globalOptimize();

        private:
            System *_system;
            Graph *_graph;

    };

}

#endif
