#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <mutex>
#include <memory>

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"

class Wall;
class Pose;
class Graph
{
    public:
        typedef std::shared_ptr<Graph> Ptr;
        typedef std::shared_ptr<const Graph> ConstPtr;

        std::mutex _graphUpdateMutex;

        Graph ();
        void addVertex (Wall*);
        std::vector<Wall*> getAllVertices ();
        void updateGraph (std::vector<Wall>);
        void dataAssociationNN (Wall&); // Simple but inaccurate Nearest Neighbour

    private:
        std::vector<Wall*> _walls;
        std::vector<Pose*> _poses;

};

#endif
