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
