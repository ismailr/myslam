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

    Graph3::Graph3() {
    }

    void Graph3::insertNode (Pose3::Ptr pose) {
        {
            std::unique_lock<std::mutex> lock (_nodeMutex);
            pose->_active = true;
            _poseMap[pose->_id] = pose;
            _path.push_back (pose->_id);
        }
    }

    void Graph3::insertNode (ObjectXYZ::Ptr object) {
        {
            std::unique_lock<std::mutex> lock (_nodeMutex);
            object->_active = true;
            _objectMap[object->_id] = object;
            _objectClassMap[object->_classid].insert (object->_id); 
        }

        int x = (int)round((object->_pose[0])*10);
        int y = (int)round((object->_pose[1])*10);
        int z = (int)round((object->_pose[2])*10);
        std::tuple<int,int,int> xyz = std::make_tuple(x,y,z);
        {
            std::unique_lock<std::mutex> lock (_gridMutex);
            _grid[xyz].insert(object->_id); 
            _gridLookup[object->_id] = xyz;
        }
    }

    int Graph3::insertPosePoseEdge (int from, int to, g2o::Isometry3 d) {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        Pose3Measurement::Ptr pm (new Pose3Measurement);
        pm->_active = true;
        pm->_from = from;
        pm->_to = to;
        pm->_measurement = d;
        _posePoseMap[pm->_id] = pm;
        return pm->_id;
    }

    int Graph3::insertPoseObjectEdge (int from, int to, Eigen::Vector3d d) {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        ObjectXYZMeasurement::Ptr om (new ObjectXYZMeasurement);
        om->_active = true;
        om->_from = from;
        om->_to = to;
        om->_measurement = d;
        _poseObjectMap[om->_id] = om;
        return om->_id;
    }

    void Graph3::resetActive () {

        for (auto it = _poseMap.begin(); it != _poseMap.end(); it++) {
            std::unique_lock<std::mutex> lock (_nodeMutex);
            (it->second)->_active = false;
        }

        for (auto it = _objectMap.begin(); it != _objectMap.end(); it++) {
            std::unique_lock<std::mutex> lock (_nodeMutex);
            (it->second)->_active = false;
        }

        for (auto it = _posePoseMap.begin(); it != _posePoseMap.end(); it++) {
            std::unique_lock<std::mutex> lock (_nodeMutex);
            (it->second)->_active = false;
        }

        for (auto it = _poseObjectMap.begin(); it != _poseObjectMap.end(); it++) {
            std::unique_lock<std::mutex> lock (_nodeMutex);
            (it->second)->_active = false;
        }
    }

    std::map<int, Pose3::Ptr> Graph3::getPoseMap () {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _poseMap;
    }

    std::map<int, ObjectXYZ::Ptr> Graph3::getObjectMap () {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _objectMap;
    }

    std::map<int, std::set<int>> Graph3::getObjectClassMap () {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _objectClassMap;
    }

    std::map<int, Pose3Measurement::Ptr> Graph3::getPosePoseMap () {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _posePoseMap;
    }

    std::map<int, ObjectXYZMeasurement::Ptr> Graph3::getPoseObjectMap () {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _poseObjectMap;
    }

    std::vector<int> Graph3::getPath () {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _path;
    }

    void Graph3::updateGrid (int id) {

        std::unique_lock<std::mutex> lock_node (_nodeMutex);
        if (_objectMap.find(id) == _objectMap.end())
            return;

        int x = (int) round (_objectMap[id]->_pose[0] * 10);
        int y = (int) round (_objectMap[id]->_pose[1] * 10);
        int z = (int) round (_objectMap[id]->_pose[2] * 10);

        auto xyz = std::make_tuple (x,y,z);

        std::unique_lock<std::mutex> lock_grid (_gridMutex);
        if (_gridLookup.find(id) != _gridLookup.end()) {
            auto _xyz = _gridLookup[id];
            if (_grid.find(_xyz) != _grid.end()) {
                auto objects = _grid[_xyz];
                if (objects.find(id) != objects.end()) {
                    objects.erase (id);
                } 

                if (objects.empty()) {
                    _grid.erase (_xyz);
                }
            } 
        } 

        _grid[xyz].insert(id);
        _gridLookup[id] = xyz;
    }

    std::set<int> Graph3::lookSurroundingCell (std::tuple<int,int,int> xyz) {

        std::set<int> result;

        int x = std::get<0>(xyz);
        int y = std::get<1>(xyz);
        int z = std::get<2>(xyz);

        std::vector<int> xs { x-2, x-1, x, x+1, x+2 };
        std::vector<int> ys { y-2, y-1, y, y+1, y+2 };
        std::vector<int> zs { z-2, z-1, z, z+1, z+2 };

        std::unique_lock<std::mutex> lock (_gridMutex);

        for (int i = 0; i < xs.size(); i++) {
            for (int j = 0; j < ys.size(); j++) {
                for (int k = 0; k < zs.size(); k++) {
                    auto cell = std::make_tuple (xs[i],ys[j],zs[k]);
                    auto it = _grid.find (cell);
                    if (it != _grid.end())
                        result.insert (_grid[cell].begin(), _grid[cell].end());
                }
            }
        }

        return result;
    }

    int Graph3::matchSurroundingCell (int classid, std::tuple<int,int,int> xyz) {
        std::set<int> result = lookSurroundingCell (xyz);
        for (auto it = result.begin(); it != result.end(); it++) {
            int objclassid;
            {
                std::unique_lock<std::mutex> lock (_nodeMutex);
                objclassid = _objectMap[*it]->_classid;
            }
                
            if (objclassid == classid) return *it;
        }

        return -1;
    }

    bool Graph3::isReady() {
        if (_poseMap.size() > 3) return true;
       return false;
    }

    std::set<int> Graph3::getDetectedObjectsFromPose (int id) {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _poseMap[id]->_detectedObjects;
    }

    std::set<int> Graph3::getPosesFromObjects (int id) {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _objectMap[id]->_seenBy;

    }

    Pose3::Ptr Graph3::getPose (int id) {
        std::unique_lock<std::mutex> lock (_nodeMutex);
        return _poseMap[id];
    }
}
