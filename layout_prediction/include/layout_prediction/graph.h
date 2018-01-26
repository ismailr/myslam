#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <mutex>
#include <memory>
#include <map>
#include <tuple>
#include <set>
#include <math.h>

#include "layout_prediction/wall.h"
#include "layout_prediction/point.h"
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
//    AngleMeasurement::Ptr createAngleMeasurement();
    PoseMeasurement2::Ptr createPoseMeasurement(int,int);
    WallMeasurement3::Ptr createWallMeasurement3(int,int);

    Wall2::Ptr data_association (Wall2::Ptr& wall);
    Wall3::Ptr data_association (Wall3::Ptr& wall);
    void optimize();
    void localOptimize(bool init);
    void localOptimize(bool init, std::map<int, std::set<int> > data);
    void optimizeIncremental();

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
    std::vector<Wall3::Ptr> _wall3DB;
    std::vector<PoseMeasurement2::Ptr> _poseMeasurementDB;
    std::vector<WallMeasurement2::Ptr> _wallMeasurementDB;
    std::vector<WallMeasurement3::Ptr> _wallMeasurementDB3;
//    std::vector<AngleMeasurement::Ptr> _angleMeasurementDB;

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

namespace MYSLAM {
    class System;
    class Graph
    {
        public:
            Graph();
            Graph(System&);
            Wall::Ptr dataAssociation (Wall::Ptr& w);
            Point::Ptr dataAssociationPoint (Point::Ptr& p);

            std::map<int, Pose::Ptr> _poseMap;
            std::map<int, Wall::Ptr> _wallMap;
            std::map<int, Point::Ptr> _pointMap;

            std::map<std::tuple<int, int>, Eigen::Vector3d> _posePoseMap;
            std::map<std::tuple<int, int>, Eigen::Vector2d> _poseWallMap;
            std::map<std::tuple<int, int>, Eigen::Vector2d> _posePointMap;

            // local optimization book keeping
            std::vector<int> _activePoses;
            std::set<int> _activeWalls;
            std::set<int> _lastActiveWalls;
            std::set<int> _activePoints;
            std::vector<std::tuple<int, int> > _activeEdges;

        private:
            System *_system;
    };

}

#endif
