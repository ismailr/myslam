#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <mutex>
#include <memory>
#include <map>
#include <tuple>

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
        void addVertex (Wall::Ptr);
        std::vector<Wall::Ptr> getAllVertices ();

    private:
        std::vector<Wall::Ptr> _walls;
        std::vector<Pose*> _poses;

        std::map <std::tuple<double, double>, std::vector<Wall::Ptr> > _local_maps;

        void addToMap (Wall::Ptr);
        void updateGraph (std::vector<Wall::Ptr>);
};

#endif
