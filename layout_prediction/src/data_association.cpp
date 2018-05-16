#include <limits>
#include <algorithm>

#include "layout_prediction/data_association.h"

#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"

namespace MYSLAM {

    DataAssociation::DataAssociation(Graph& graph)
        :_graph (&graph)
    {

    }

    std::vector<int> DataAssociation::associate (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data) {

        // ids container that will be returned
        // id = -1 indicates new object
        // id = -2 indicates possibility of object already present in map
        // following algorithm should replace all id = -2
        // either with -1 or associated object id
        std::vector<int> objectsids (data.size(), -1);

        // handle single and double object detection specially
        if (data.size() == 0) return objectsids;

        // container to store path candidates
        // and distance values
        std::vector<std::vector<int> > candidates;

        // prepare input and data (map)
        std::vector<Eigen::Vector3d> measurements;
        std::vector<int> classids;
        std::vector<double> dz; // distance between measurements
        for (int i = 0; i < data.size(); i++) {
            measurements.push_back (std::get<1>(data[i]));
            classids.push_back (std::get<0>(data[i]));

            if (i < data.size() - 1) {
                double d = calculateDistance (std::get<1>(data[i]), std::get<1>(data[i+1]));
                dz.push_back (d);
            }

            // check if classids[i] is in the map
            // if yes, put -2 in objectsids[i]
//            if (_graph->_objectClassMap[classids[i]].size() > 0)
//                objectsids[i] = -2;
        }

        // very likely map is empty (maybe beginning of operation)
//        if (std::all_of(objectsids.begin(), objectsids.end(), [](int i){return i == -1;})) 
//            return objectsids;

        // fill first candidates with objects from _objectClassMap[classids[0]]
        // if _objectClassMap[classids[0]] is empty, put -1
        if (_graph->_objectClassMap[classids[0]].empty()) {
            std::vector<int> _candidates;
            _candidates.push_back (-1);
            candidates.push_back (_candidates);
        } else {
            for (auto it = _graph->_objectClassMap[classids[0]].begin(); 
                    it != _graph->_objectClassMap[classids[0]].end();
                   it ++) {
                std::vector<int> _candidates;
                _candidates.push_back (*it);
                candidates.push_back (_candidates);
            } 
        }

        std::vector<double> dcandidates (candidates.size(), 0.0); 

        for (int i = 1; i < classids.size(); i++) {

            std::set<int> to = _graph->_objectClassMap[classids[i]];
            if (to.empty()) to.insert (-1); 

            // exhaust all possible paths from nodes[i] to nodes[i+1]
            // and weight them with distances

            int loop = candidates.size(); // fix the number of iteration
            for (int j = 0; j < loop; j++) {

                Eigen::Vector3d o1;
                if (candidates[j].back() == -1) {
                    SE2 p; p.fromVector(pose->_pose);
                    SE2 m; m.fromVector(measurements[i-1]);
                    o1 = (p*m).toVector();
                } else {
                    o1 = _graph->_objectMap[candidates[j].back()]->_pose;
                }

                double shortest = std::numeric_limits<double>::max();
                int nextnode; // next shortest node from o1
                for (auto jt = to.begin(); jt != to.end(); jt++) {

                    Eigen::Vector3d o2;
                    if (*jt != -1)  {
                        auto kt = std::find (candidates[j].begin(), candidates[j].end(), *jt);
                        if (kt != candidates[j].end()) continue; // prevent path to loop
                        o2 = _graph->_objectMap[*jt]->_pose;
                    } else {
                        SE2 p; p.fromVector(pose->_pose);
                        SE2 m; m.fromVector(measurements[i]);
                        o2 = (p*m).toVector();
                    }

                    double d = calculateDistance (o1, o2);

                    if (d == 0.0) continue; // if o1 = o2, discard

                    double diff = std::abs (d - dz[i-1]);

                    if (diff < shortest) {
                        shortest = diff;
                        nextnode = *jt;
                    }
                }

//                std::cout << "******* " << shortest << std::endl;
                if (shortest > 0.02) {
                    candidates[j].push_back (-1);
                } else {
                    candidates[j].push_back (nextnode);
                }

                dcandidates [j] += shortest; 
            }
        }

        double shortest = std::numeric_limits<double>::max();
        for (int i = 0; i < candidates.size(); i++){
            if (dcandidates[i] < shortest) {
                shortest = dcandidates[i];
                objectsids = candidates[i];
            }
        }

        return objectsids;
    }

    double DataAssociation::calculateDistance (Wall::Ptr w1, Wall::Ptr w2) {
        return (double)(w1->_line.xx - w2->_line.xx).norm();
    }

    double DataAssociation::calculateDistance (Wall::Ptr w, Object::Ptr o) {
        Eigen::Vector2d p (o->_pose[0], o->_pose[1]);
        return (double)(w->_line.xx-p).norm();
    }

    double DataAssociation::calculateDistance (Object::Ptr o1, Object::Ptr o2) {
        Eigen::Vector2d p1 (o1->_pose[0], o1->_pose[1]);
        Eigen::Vector2d p2 (o2->_pose[0], o2->_pose[1]);
        return (double)(p1-p2).norm();
    }

    double DataAssociation::calculateDistance (Eigen::Vector3d o1, Eigen::Vector3d o2) {
        Eigen::Vector2d p1 (o1[0], o1[1]);
        Eigen::Vector2d p2 (o2[0], o2[1]);
        return (double)(p1-p2).norm();
    }

    double DataAssociation::calculateTotalDistance (std::vector<int> sequence) {

        int length = sequence.size();
        double dist = 0.0;

        for (int i = 0; i < length-1; i++) {

            Object::Ptr o1 = _graph->_objectMap[sequence[i]];
            Object::Ptr o2 = _graph->_objectMap[sequence[i+1]];

            dist += calculateDistance (o1, o2);
        }

        return dist;
    }

    double DataAssociation::calculateTotalDistance (std::vector<Eigen::Vector3d> sequence) {

        int length = sequence.size();
        double dist = 0.0;

        for (int i = 0; i < length-1; i++) {

            dist += calculateDistance (sequence[i], sequence[i+1]);
        }

        return dist;
    }

    double DataAssociation::calculateDiff (std::vector<int> s1, std::vector<int> s2) {

        double d1 = calculateTotalDistance (s1);
        double d2 = calculateTotalDistance (s2);
        return std::abs (d1-d2);
    }
}
