#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <map>
#include <tuple>
#include <set>

#include "layout_prediction/wall.h"
#include "layout_prediction/point.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/object.h"

namespace MYSLAM {
    class System;
    class Graph
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Graph();
            Graph(System&);
            Wall::Ptr dataAssociation (Wall::Ptr& w);
            bool dataAssociationEKF (int poseid, Wall::Ptr& w, const Eigen::Vector2d& z);
            Point::Ptr dataAssociationPoint (Point::Ptr& p);

            void insertNode (Pose::Ptr);
            void insertNode (Wall::Ptr);
            void insertNode (Object::Ptr);
            void insertPoseWallEdge (std::tuple<int,int>, Eigen::Vector2d);
            void insertPoseObjectEdge (std::tuple<int,int>, Eigen::Vector3d);

            std::map<int, Pose::Ptr> _poseMap;
            std::map<int, Wall::Ptr> _wallMap;
            std::map<int, Object::Ptr> _objectMap;
            std::map<int, Point::Ptr> _pointMap;

            // for loop closure
            std::vector<int> _poseList;

            // measurement
            std::map<std::tuple<int, int>, Eigen::Vector3d> _posePoseMap;
            std::map<std::tuple<int, int>, Eigen::Vector2d> _poseWallMap;
            std::map<std::tuple<int, int>, Eigen::Vector3d> _poseObjectMap;
            std::map<std::tuple<int, int>, Eigen::Vector2d> _posePointMap;

            // local optimization book keeping
            std::vector<int> _activePoses;
            std::set<int> _activeWalls;
            std::set<int> _activeObjects;
            std::set<int> _lastActiveWalls;
            std::set<int> _lastActiveObjects;
            std::set<int> _activePoints;
            std::vector<std::tuple<int, int> > _activeEdges;
            std::vector<std::tuple<int, int> > _activePoseObjectEdges;

        private:
            System *_system;
    };

}

#endif
