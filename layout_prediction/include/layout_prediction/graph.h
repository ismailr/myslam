#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"

class Wall;
class Pose;
class Graph
{
    public:
        Graph ();
        void addVertex (Wall*);
        std::vector<Wall*> getAllVertices ();

    private:
        std::vector<Wall*> _walls;
        std::vector<Pose*> _poses;

};

#endif
