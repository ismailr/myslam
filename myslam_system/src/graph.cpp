#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <limits>

#include "myslam_system/graph.h"

using namespace g2o;

namespace MYSLAM {
    Graph::Graph(){}; // simulation

    void Graph::insertNode (Pose::Ptr pose) {
        _poseMap[pose->_id] = pose;
        _activePoses.push_back (pose->_id);
        pose->_active = true;
        _path.push_back (pose->_id);
    }

    void Graph::insertNode (Wall::Ptr wall) {
        _wallMap[wall->_id] = wall;
        _activeWalls.insert (wall->_id);
//        wall->_active = true;
    }

    void Graph::insertNode (Object::Ptr object) {
        _objectMap[object->_id] = object;
        _activeObjects.insert (object->_id);
        _objectClassMap[object->_classid].insert (object->_id); 
        object->_active = true;
    }

    void Graph::insertPoseWallEdge (std::tuple<int,int> e, Eigen::Vector2d d) {
        _poseWallMap [e] = d;
        _activeEdges.push_back (e);
    }

    void Graph::insertPoseObjectEdge (std::tuple<int,int> e, Eigen::Vector3d d) {
        _poseObjectMap [e] = d;
        _activePoseObjectEdges.push_back (e);
    }

    void Graph::resetActive () {

        for (auto it = _poseMap.begin(); it != _poseMap.end(); it++) {
            (it->second)->_active = false;
        }

        for (auto it = _wallMap.begin(); it != _wallMap.end(); it++) {
            (it->second)->_active = false;
        }

        for (auto it = _objectMap.begin(); it != _objectMap.end(); it++) {
            (it->second)->_active = false;
        }
    }

    Graph3::Graph3() {}

    void Graph3::insertNode (Pose3::Ptr pose) {
        {
            std::unique_lock<std::mutex> lock (nodeMutex);
            pose->_active = true;
            _poseMap[pose->_id] = pose;
            _path.push_back (pose->_id);
        }
    }

    void Graph3::insertNode (ObjectXYZ::Ptr object) {
        std::unique_lock<std::mutex> lock (nodeMutex);
        object->_active = true;
        _objectMap[object->_id] = object;
        _objectClassMap[object->_classid].insert (object->_id); 
    }

    void Graph3::insertPosePoseEdge (int from, int to, g2o::Isometry3 d) {
        std::unique_lock<std::mutex> lock (nodeMutex);
        Pose3Measurement::Ptr pm (new Pose3Measurement);
        pm->_active = true;
        pm->_from = from;
        pm->_to = to;
        pm->_measurement = d;
        _posePoseMap[pm->_id] = pm;
    }

    void Graph3::insertPoseObjectEdge (int from, int to, Eigen::Vector3d d) {
        std::unique_lock<std::mutex> lock (nodeMutex);
        ObjectXYZMeasurement::Ptr om (new ObjectXYZMeasurement);
        om->_active = true;
        om->_from = from;
        om->_to = to;
        om->_measurement = d;
        _poseObjectMap[om->_id] = om;
    }

    void Graph3::resetActive () {

        for (auto it = _poseMap.begin(); it != _poseMap.end(); it++) {
            std::unique_lock<std::mutex> lock (nodeMutex);
            (it->second)->_active = false;
        }

        for (auto it = _objectMap.begin(); it != _objectMap.end(); it++) {
            std::unique_lock<std::mutex> lock (nodeMutex);
            (it->second)->_active = false;
        }

        for (auto it = _posePoseMap.begin(); it != _posePoseMap.end(); it++) {
            std::unique_lock<std::mutex> lock (nodeMutex);
            (it->second)->_active = false;
        }

        for (auto it = _poseObjectMap.begin(); it != _poseObjectMap.end(); it++) {
            std::unique_lock<std::mutex> lock (nodeMutex);
            (it->second)->_active = false;
        }
    }
}
