#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <mutex>
#include <memory>
#include <map>
#include <tuple>
#include <set>

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
        void addVertex (Wall::Ptr wallPtr); 
        void addVertex (Pose::Ptr posePtr);
        void addEdgePose2Pose (std::tuple<int, int>, SE2&); 
        void addEdgePose2Wall (std::tuple<int, int>, double*); 

        Pose::Ptr& getPoseVertex (int id) { return _poses[id]; };
        Wall::Ptr& getWallVertex (int id) { return _walls[id]; };

        bool localOptimize (int);  // local optimization from poseid down to poseid - 3 (4 poses)
        int generateIdForVertex() { return Graph::id++; };

    private:
        static unsigned long id;
        std::map<int, Wall::Ptr> _walls; // id -> wall object
        std::map<int, Pose::Ptr> _poses; // id -> pose object
        std::map<std::tuple<int, int>, PoseMeasurement::Ptr> _pose2pose; // pose_i to pose_{i + 1} edge -> odometry
        std::map<std::tuple<int, int>, WallMeasurement::Ptr> _pose2wall; // pose to wall edge -> measurement

        std::map<int, std::vector<int> > _pose2wallmap;  // pose id -> all the ids of walls observed
        std::vector<int> _poseIds;
        std::vector<int> _wallIds;

        void updateGraph (std::vector<Wall::Ptr>);
        void setAnchor (int poseId, bool set);
        void occupancyGrid (int poseId);
        int associateData (int wallId, std::vector<int> ref);
};

class Graph2
{
    public:
    Graph2 ();

    Wall2::Ptr wall_alloc() { Wall2::Ptr w (new Wall2); return w; }; 
    Pose2::Ptr pose_alloc() { Pose2::Ptr p (new Pose2()); return p; };

    typedef WallMeasurement2::Ptr Wm;
    typedef PoseMeasurement2::Ptr Pm;
    Wm wallmeasurement_alloc() { Wm wm (new WallMeasurement2); return wm; };
    Pm posemeasurement_alloc() { Pm pm (new PoseMeasurement2); return pm; };

    private:
        std::map<Eigen::Vector2d, Wall2::Ptr> wallDB;
};

#endif
