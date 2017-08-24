#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include "layout_prediction/graph.h"
#include "layout_prediction/system.h"

class Graph;
class System;
class Optimizer
{
    public:
        Optimizer (System&, Graph&);
        void run ();

    private:
        Graph *_graph;
        System *_system;
        g2o::SparseOptimizer* _optimizer;

};

#endif
