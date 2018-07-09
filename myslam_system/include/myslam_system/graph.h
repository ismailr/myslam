#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <map>
#include <tuple>
#include <set>

#include "myslam_system/wall.h"
#include "myslam_system/pose.h"
#include "myslam_system/object.h"

namespace MYSLAM {
    class Graph
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Graph();

            void insertNode (Pose::Ptr);
            void insertNode (Wall::Ptr);
            void insertNode (Object::Ptr);
            void insertPoseWallEdge (std::tuple<int,int>, Eigen::Vector2d);
            void insertPoseObjectEdge (std::tuple<int,int>, Eigen::Vector3d);

            void resetActive ();

            std::map<int, Pose::Ptr> _poseMap;
            std::map<int, Wall::Ptr> _wallMap;
            std::map<int, Object::Ptr> _objectMap;
            std::map<int, std::set<int> > _objectClassMap;

            // path
            std::vector<int> _path;

            // measurement
            std::map<std::tuple<int, int>, Eigen::Vector3d> _posePoseMap;
            std::map<std::tuple<int, int>, Eigen::Vector2d> _poseWallMap;
            std::map<std::tuple<int, int>, Eigen::Vector3d> _poseObjectMap;

            // local optimization book keeping
            std::vector<int> _activePoses;
            std::set<int> _activeWalls;
            std::set<int> _activeObjects;
            std::set<int> _lastActiveWalls;
            std::set<int> _lastActiveObjects;
            std::vector<std::tuple<int, int> > _activeEdges;
            std::vector<std::tuple<int, int> > _activePoseObjectEdges;
    };
}

#endif
