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

    int createPose();
    int createWall();
    int registerWall(Wall2::Ptr& w);
    int createPoseMeasurement();
    int createWallMeasurement();

    Pose2::Ptr getPose(int id) { return _poseDB[id]; };
    Wall2::Ptr getWall(int id) { return _wallDB[id]; };
    PoseMeasurement2::Ptr getPoseMeasurement (int id) { return _poseMeasurementDB[id]; };
    WallMeasurement2::Ptr getWallMeasurement (int id) { return _wallMeasurementDB[id]; };

    void optimize();

    private:
    static int globalId;
    g2o::SparseOptimizer *_optimizer;
    g2o::HyperGraph::VertexSet *_vertexSet;

    std::map<int, Pose2::Ptr> _poseDB;
    std::map<int, Wall2::Ptr> _wallDB;
    std::map<int, PoseMeasurement2::Ptr> _poseMeasurementDB;
    std::map<int, WallMeasurement2::Ptr> _wallMeasurementDB;

    int data_association (int id, std::vector<int> walls);
};

#endif
