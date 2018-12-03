#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include "myslam_system/graph.h"

namespace MYSLAM {
    class Optimizer
    {
        public:
            Optimizer(MYSLAM::Graph&); // simulation
            void localOptimize();
            void localOptimize3();
            void localOptimize2(Pose::Ptr pose);
            void globalOptimize();

        private:
            Graph *_graph;
    };

    class Optimizer3 {

        public:
            Optimizer3 (Graph3&);
            void localOptimize(int mode);

        private:
            Graph3 *_graph;
    };
}

#endif
