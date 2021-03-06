#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <map>
#include <tuple>
#include <set>
#include <mutex>

#include "myslam_system/types_myslam.h"

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

    class Graph3 {

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Graph3();

            std::map<int, Pose3::Ptr> _poseMap;
            std::map<int, ObjectXYZ::Ptr> _objectMap;
            std::map<int, std::set<int>> _objectClassMap;
            std::map<int, Pose3Measurement::Ptr> _posePoseMap;
            std::map<int, ObjectXYZMeasurement::Ptr> _poseObjectMap;

            std::vector<int> _path;

            void insertNode (Pose3::Ptr pose);
            void insertNode (ObjectXYZ::Ptr object); 
            int insertPosePoseEdge (int from, int to, g2o::Isometry3 d); 
            int insertPoseObjectEdge (int from, int to, Eigen::Vector3d);
            void resetActive ();

            std::map<int, Pose3::Ptr> getPoseMap ();
            std::map<int, ObjectXYZ::Ptr> getObjectMap ();
            std::map<int, std::set<int>> getObjectClassMap ();
            std::map<int, Pose3Measurement::Ptr> getPosePoseMap ();
            std::map<int, ObjectXYZMeasurement::Ptr> getPoseObjectMap ();
            std::vector<int> getPath ();
            std::set<int> getDetectedObjectsFromPose (int id);
            std::set<int> getPosesFromObjects (int id);
            Pose3::Ptr getPose (int id);

            std::mutex _nodeMutex;

            bool isReady();

        private:

            std::mutex _gridMutex;
            std::map<std::tuple<int,int,int>,std::set<int>> _grid;
            std::map<int, std::tuple<int,int,int>> _gridLookup;

        public:
            void updateGrid (int objectId);
            std::map<std::tuple<int,int,int>,std::set<int>> getGrid () { return _grid; };
            std::map<int, std::tuple<int,int,int>> getGridLookup () { return _gridLookup; };
            std::set<int> lookSurroundingCell (std::tuple<int,int,int>);
            int matchSurroundingCell (int, std::tuple<int,int,int>);
    };
}

#endif
