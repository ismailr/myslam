#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <mutex>

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"

class Wall;
class Pose;
class Graph
{
    public:
        std::mutex _graphUpdateMutex;

        Graph ();
        void addVertex (Wall*);
        std::vector<Wall*> getAllVertices ();
        void updateGraph (std::vector<Wall>);

    private:
        std::vector<Wall*> _walls;
        std::vector<Pose*> _poses;

};

#endif
