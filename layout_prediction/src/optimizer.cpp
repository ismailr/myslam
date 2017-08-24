#include "layout_prediction/optimizer.h"
#include "layout_prediction/graph.h"

Optimizer::Optimizer (System& system, Graph& graph)
    :_system (&system), _graph (&graph) {
    _system->setOptimizer (*this);
}

void Optimizer::run (){
    while(1)
    {
    }
}
