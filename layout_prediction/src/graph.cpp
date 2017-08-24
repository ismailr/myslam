#include "layout_prediction/graph.h"
#include "layout_prediction/wall.h"

Graph::Graph (){}

void Graph::addVertex (Wall* wall){
    _walls.push_back (wall);
}

std::vector<Wall*> Graph::getAllVertices ()
{
    return _walls;
}

void Graph::updateGraph (std::vector<Wall> newGraph)
{
    std::unique_lock<std::mutex> lock (_graphUpdateMutex);
    for (int i = 0; i < newGraph.size(); ++i)
        // _walls[newGrah[i].id] = newGraph[i];

    lock.unlock();
}


