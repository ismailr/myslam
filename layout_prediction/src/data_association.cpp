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
        std::vector<int> objectsids (data.size(), -1);
        if (data.size() <= 1) return objectsids;

        // break the input
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
        }

        // first, handle new object, i.e. the one with no class in map
        std::set<int> newClasses;
        for (int i = 0; i < classids.size(); i++) {

            auto it = std::find (newClasses.begin(), newClasses.end(), classids[i]);
            if (_graph->_objectClassMap[classids[i]].size() == 0 || it != newClasses.end() ) {

                newClasses.insert (classids[i]);

                Object::Ptr o (new Object);
                SE2 robotPose; robotPose.fromVector (pose->_pose);
                SE2 measurement; measurement.fromVector (measurements[i]);
                o->_pose = (robotPose * measurement).toVector();
                o->_classid = classids[i];
                _graph->insertNode (o);
            } 
        }

        // candidate containers (only n shortest paths will be recorded)
        std::set<int> _from = _graph->_objectClassMap[classids[0]];
        std::set<int> _to = _graph->_objectClassMap[classids[1]];
        int npaths = _from.size() * _to.size();

        int n;
        npaths > 2 ? n = 3 : n = npaths; 
        std::vector<std::vector<int> > candidates (n);
        std::vector<double> dcandidates (n, 0.0); // weights

        // get n candidates
        // calculated from level 0 and level 1 nodes
        std::vector<std::tuple<double,int,int> > path_candidates;
        for (auto it = _from.begin(); it != _from.end(); it++) {

            Object::Ptr o1 = _graph->_objectMap[*it];
            for (auto jt = _to.begin(); jt != _to.end(); jt++) {

                Object::Ptr o2 = _graph->_objectMap[*jt];
                double d = calculateDistance (o1, o2);

                if (d == 0) continue; // if o1 = o2, discard

                double diff = std::abs (d - dz[0]);

                path_candidates.push_back (std::make_tuple (diff, *it, *jt));
            }

            std::sort ( begin(path_candidates), 
                        end(path_candidates),
                        [](auto const &t1, auto const &t2) {
                            return get<0>(t1) < get<0>(t2);
                        });

            for (int j = 0; j < n; j++) {

                int f = std::get<1>(path_candidates[j]);
                int t = std::get<2>(path_candidates[j]);
                double d = std::get<0>(path_candidates[j]);

                candidates[j].push_back (f); 
                candidates[j].push_back (t);
                dcandidates[j] = d;
            }
        }

        // iterate through _classObjectMap[classids[i]]
        for (int i = 2; i < classids.size(); i++) {

            std::set<int> to = _graph->_objectClassMap[classids[i]];

            // exhaust all possible paths from "from" to "to"
            // and weight them with distances
            // "from" is now candidates[i].back()
            for (auto it = candidates.begin(); it != candidates.end(); it++) {

                Object::Ptr o1 = _graph->_objectMap[it->back()];
                double shortest = std::numeric_limits<double>::max();
                int nextnode; // next shortest node from o1
                for (auto jt = to.begin(); jt != to.end(); jt++) {

                    Object::Ptr o2 = _graph->_objectMap[*jt];
                    double d = calculateDistance (o1, o2);

                    if (d == 0) continue; // if o1 = o2, discard
                    auto kt = std::find (it->begin(), it->end(), o2->_id);
                    if (kt != it->end()) continue;

                    double diff = std::abs (d - dz[i-1]);

                    if (diff < shortest) {
                        shortest = diff;
                        nextnode = *jt;
                    }
                }

                it->push_back (nextnode);
                dcandidates [it - candidates.begin()] += shortest; 
            }
        }

        std::cout << "CANDIDATES: " << std::endl;
        for (int k = 0; k < candidates.size(); k++) {
            std::cout << dcandidates[k] << ": ";
            for (int l = 0; l < classids.size(); l++) {
                std::cout << candidates[k][l] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;
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
