#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <limits>

#include "layout_prediction/graph.h"
#include "layout_prediction/settings.h"

using namespace g2o;

namespace MYSLAM {
    Graph::Graph(){}; // simulation
    Graph::Graph(System& system):_system (&system) {};

    Wall::Ptr Graph::dataAssociation (Wall::Ptr& w)
    {
        double _dr_ = 1.0;
        double _dt_ = 10.0 * M_PI/180.0;

        double& xx = w->_line.xx[0];
        double& xy = w->_line.xx[1];

        double r = sqrt (xx*xx + xy*xy);
        double t = atan2 (xy,xx);

        double drmin = std::numeric_limits<double>::max();
        double dtmin = std::numeric_limits<double>::max();
        int idmin = w->_id;

        std::set<int> walls;
        walls.insert (_lastActiveWalls.begin(), _lastActiveWalls.end());
        walls.insert (_activeWalls.begin(), _activeWalls.end());

//        for (std::map<int, Wall::Ptr>::iterator it = _wallMap.begin();
//                it != _wallMap.end(); it++)
        for (std::set<int>::iterator it = walls.begin();
                it != walls.end(); it++)
        {
            double& xxref = _wallMap[*it]->_line.xx[0];
            double& xyref = _wallMap[*it]->_line.xx[1];
            double rref = sqrt (xxref*xxref + xyref*xyref);
            double tref = atan2 (xyref,xxref);

            double dr = std::abs (r - rref);
            double dt = std::abs (t - tref);

            if (dr < drmin && dt < dtmin) {
                drmin = dr;
                dtmin = dt;
                idmin = *it;
            }
        }

        if (drmin < _dr_ && dtmin < _dt_) {
            Eigen::Vector2d p = w->_line.p;
            Eigen::Vector2d q = w->_line.q;
            _wallMap[idmin]->updateSegment (p,q);

            return _wallMap[idmin];
        }

        // new landmark
        _wallMap[w->_id] = w;
//        _wallMap[w->_id]->updateSegment (p, q);
//        _activeWalls.push_back (w->_id);
        return w;
    }

    bool Graph::dataAssociationEKF (int poseid, Wall::Ptr& w, const Eigen::Vector2d& z) {

        if (_wallMap.empty()) return true;

        SE2 pose; pose.fromVector (_poseMap[poseid]->_pose);

        double pmin = std::numeric_limits<double>::max();
        int id;
        Eigen::Matrix2d cov;
        bool candidate = false;

        // min log likelihood
//        std::cout << std::endl;
//        std::cout << "****************************" << std::endl;
//        std::cout << "MEASURED WALL: " << (pose * z).transpose() << std::endl;
        for (std::map<int, Wall::Ptr>::iterator it = _wallMap.begin();
                it != _wallMap.end(); it++)
        {
            // sensor model
            Wall::Ptr _w = it->second;

            Eigen::Vector2d z_hat = pose.inverse() * _w->_line.xx;
            Eigen::Vector2d e = z - z_hat;

//            double den = 2 * M_PI * sqrt(_w->cov.determinant());
//            double num = std::exp(-0.5 * e.transpose() * _w->cov.inverse() * e);
//            double p = num/den;
            double p = log(_w->cov.determinant()) + e.transpose() * _w->cov.inverse() * e;

//            std::cout << "+\t" << _w->_line.xx.transpose();
            if (p < pmin) {
//                std::cout << " ---> CANDIDATE" << std::endl;
                pmin = p;
                id = _w->_id;
                cov = _w->cov;
                candidate = true;
            }
//            std::cout << std::endl;
        }

        double p_thres = std::numeric_limits<double>::min();
        if (candidate) {
            double stdevx = sqrt (cov(0,0));
            double stdevy = sqrt (cov(1,1));
            double xmean = _wallMap[id]->_line.xx[0];
            double ymean = _wallMap[id]->_line.xx[1];
//            Eigen::Vector2d stdv (0.2 * stdevx - _wallMap[id]->_line.xx[0], 0.2 * stdevy - _wallMap[id]->_line.xx[1]);
            Eigen::Vector2d stdv (0.1 * xmean, 0.1 * ymean);
    //        double den_thres = 2 * M_PI * sqrt (cov.determinant());
    //        double num_thres = std::exp (-0.5 * stdv.transpose() * cov.inverse() * stdv);
    //        double p_thres = num_thres/den_thres;
            p_thres = log(cov.determinant()) + stdv.transpose() * cov.inverse() * stdv;
        }

        if (pmin < p_thres) {
            w = _wallMap[id];
//            std::cout   << "WINNER: " << w->_line.xx.transpose() 
//                        << " WITH P: " << pmin << " AND THRESHOLD IS: " << p_thres << std::endl;
//            std::cout << std::endl;
            return false;
           
        } else {
//            std::cout << "NEW LANDMARK" << std::endl;
//            std::cout << "****************************" << std::endl;
            return true;
        }
    }

    void Graph::insertNode (Pose::Ptr pose) {
        _poseMap[pose->_id] = pose;
        _activePoses.push_back (pose->_id);
    }

    void Graph::insertNode (Wall::Ptr wall) {
        _wallMap[wall->_id] = wall;
        _activeWalls.insert (wall->_id);
    }

    void Graph::insertNode (Object::Ptr object) {
        _objectMap[object->_id] = object;
        _activeObjects.insert (object->_id);
        _objectClassMap[object->_classid].push_back (object->_id); 
    }

    void Graph::insertPoseWallEdge (std::tuple<int,int> e, Eigen::Vector2d d) {
        _poseWallMap [e] = d;
        _activeEdges.push_back (e);
    }

    void Graph::insertPoseObjectEdge (std::tuple<int,int> e, Eigen::Vector3d d) {
        _poseObjectMap [e] = d;
        _activePoseObjectEdges.push_back (e);
    }
}
