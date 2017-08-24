#include "layout_prediction/optimizer.h"
#include "layout_prediction/graph.h"

Optimizer::Optimizer (){
    _graph = new Graph ();

}

void Optimizer::run (){
    while(1)
    {
    }
}

void Optimizer::attachTo (System* system)
{
    _system = system;
}
