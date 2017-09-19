#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <mutex>
#include <memory>
#include <map>
#include <tuple>

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/wall_measurement.h"

class Wall;
class Pose;
class Graph
{
    public:
        typedef std::shared_ptr<Graph> Ptr;
        typedef std::shared_ptr<const Graph> ConstPtr;

        std::mutex _graphUpdateMutex;

        Graph ();
        void addVertex (Wall::Ptr wallPtr) { _wallmap[wallPtr->id()] = wallPtr; };
        void addVertex (Pose::Ptr posePtr) { _posemap[posePtr->id()] = posePtr; };
        void addEdgePose2Pose (std::tuple<int, int>, SE2&); 
        void addEdgePose2Wall (std::tuple<int, int>, double*); 

        Pose::Ptr& getPoseVertex (int id) { return _posemap[id]; };
        Wall::Ptr& getWallVertex (int id) { return _wallmap[id]; };

        std::vector<Wall::Ptr> getAllVertices ();
        bool localOptimize (int);  // local optimization from poseid down to poseid - 3 (4 poses)

    private:
        std::map<int, Wall::Ptr> _wallmap; // id -> wall object
        std::map<int, Pose::Ptr> _posemap; // id -> pose object
        std::map<int, std::vector<int> > _pose2wallmap;  // pose id -> all the ids of walls observed
        std::map<std::tuple<int, int>, PoseMeasurement::Ptr> _pose2pose; // pose_i to pose_{i + 1} edge -> odometry
        std::map<std::tuple<int, int>, WallMeasurement::Ptr> _pose2wall; // pose to wall edge -> measurement

        std::vector<Wall::Ptr> _walls; // todo: delete
        std::vector<Pose::Ptr> _poses; // todo: delete

        void updateGraph (std::vector<Wall::Ptr>);
        void setAnchor (int poseId, bool set);
};

#endif
