#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <map>
#include <tuple>
#include <set>

#include "myslam_data_structures/pose.h"
#include "myslam_data_structures/object.h"

namespace MYSLAM {
    class Graph2D
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Graph2D();

            void insertNode (Pose2D::Ptr);
//            void insertNode (Wall::Ptr);
            void insertNode (Object2D::Ptr);
//            void insertPoseWallEdge (std::tuple<int,int>, Eigen::Vector2d);
            void insertPoseObjectEdge (std::tuple<int,int>, Eigen::Vector3d);

            void resetActive ();

            std::map<int, Pose2D::Ptr> _poseMap;
//            std::map<int, Wall::Ptr> _wallMap;
            std::map<int, Object2D::Ptr> _objectMap;
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
