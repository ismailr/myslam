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
}
