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

namespace MYSLAM {
    class Wall;
    class Pose;
    class System;
    class Graph
    {
        public:
            Graph();
            Graph(System&);
            Wall::Ptr dataAssociation (Wall::Ptr& w);
            Wall::Ptr dataAssociation2 (Wall::Ptr& w);
            bool dataAssociationEKF (int poseid, Wall::Ptr& w, const Eigen::Vector2d& z);
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
