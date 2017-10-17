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

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph.h>

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
    int requestId () { return Graph2::globalId++; };

    Pose2::Ptr createPose();
    Wall2::Ptr createWall();
    void registerWall (Wall2::Ptr& wall);
    PoseMeasurement2::Ptr createPoseMeasurement();
    WallMeasurement2::Ptr createWallMeasurement();

    Wall2::Ptr data_association (Wall2::Ptr& wall);
    void optimize();

    private:
    static int globalId;
    const float GRID_STEP = 5.0;
    const float ANGLE_STEP = 30.0;

    int _pid;
    int _wid;
    int _pmid;
    int _wmid;

    g2o::SparseOptimizer *_optimizer;
    g2o::HyperGraph::VertexSet *_vertexSet;

    std::vector<Pose2::Ptr> _poseDB;
    std::vector<Wall2::Ptr> _wallDB;
    std::vector<PoseMeasurement2::Ptr> _poseMeasurementDB;
    std::vector<WallMeasurement2::Ptr> _wallMeasurementDB;

    std::map<std::tuple<int,int>, Wall2::Ptr> _grid; // for data association
};

#endif
