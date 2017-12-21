#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include "layout_prediction/system.h"
#include "layout_prediction/graph.h"

namespace MYSLAM {
    class Optimizer
    {
        public:
            Optimizer(System&, Graph&);
            void localOptimize();
            void loopClosure();
            void globalOptimize();

        private:
            System *_system;
            Graph *_graph;

    };

}

#endif
