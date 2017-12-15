#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <mutex>
#include <memory>
#include <map>
#include <tuple>
#include <set>
#include <math.h>

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/wall_measurement.h"
#include "layout_prediction/angle_measurement.h"
#include "layout_prediction/system.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.h>
#include <g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h>
#include <slam_parser/interface/parser_interface.h>

class Wall;
class Pose;
class System2;
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
    Wall3::Ptr createWall3();
    Pose2::Ptr createPoseWithId();
    Wall2::Ptr createWallWithId();
    Wall3::Ptr createWall3WithId();
    void registerWall (Wall2::Ptr& wall);
    void registerWallWithId (Wall2::Ptr& wall);
    void registerWall3 (Wall3::Ptr& wall);
    PoseMeasurement2::Ptr createPoseMeasurement();
    WallMeasurement2::Ptr createWallMeasurement();
    WallMeasurement3::Ptr createWallMeasurement3();
    AngleMeasurement::Ptr createAngleMeasurement();
    PoseMeasurement2::Ptr createPoseMeasurement(int,int);
    WallMeasurement3::Ptr createWallMeasurement3(int,int);

    Wall2::Ptr data_association (Wall2::Ptr& wall);
    Wall3::Ptr data_association (Wall3::Ptr& wall);
    void optimize();
    void localOptimize(bool init);
    void localOptimize(bool init, std::map<int, std::set<int> > data);

    std::vector<Wall2::Ptr> getWallDB() { return _wallDB; };
    std::vector<Wall3::Ptr> getWallDB3() { return _wallDB3; };

    void setSystem (System2* system) { _system = system; };


    private:
    System2 *_system;
    static int globalId;
    const float GRID_STEP = 3.0;
    const float ANGLE_STEP = 5.0 * M_PI/180.0;
    const float CENTER_THRESHOLD = 3.0;
    const float THRESHOLD = 0.5;

    int _pid;
    int _wid;
    int _pmid;
    int _wmid;

    g2o::SparseOptimizer *_optimizer;
    g2o::HyperGraph::VertexSet *_vertexSet;

    g2o::SparseOptimizerIncremental *_incOptimizer;
    g2o::G2oSlamInterface *_slamInterface;

    std::vector<Pose2::Ptr> _poseDB;
    std::vector<Wall2::Ptr> _wallDB;
    std::vector<Wall3::Ptr> _wallDB3;
    std::vector<PoseMeasurement2::Ptr> _poseMeasurementDB;
    std::vector<WallMeasurement2::Ptr> _wallMeasurementDB;
    std::vector<WallMeasurement3::Ptr> _wallMeasurementDB3;
    std::vector<AngleMeasurement::Ptr> _angleMeasurementDB;

    std::map<int, Pose2::Ptr> _poseMap;
    std::map<int, Wall3::Ptr> _wallMap;
    std::map<std::tuple<int,int>, PoseMeasurement2::Ptr> _pose2poseMap;
    std::map<std::tuple<int,int>, WallMeasurement3::Ptr> _pose2wallMap;

    std::vector<int> _currentPoses;
    std::vector<int> _currentWalls;

    std::map<std::tuple<int,int>, Wall2::Ptr> _grid; // for data association
    std::map<std::tuple<int,int>, Wall3::Ptr> _grid3; // for data association
    double calculate_euclidean_distance (Eigen::Vector2d p, Eigen::Vector2d q);

    std::vector<int> currentNodes;
};

#endif
