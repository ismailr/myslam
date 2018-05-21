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

        // handle single and double object detection specially
        if (data.size() == 0) return objectsids;


        if (data.size() == 1) {
            int result = associate (pose, std::get<0>(data[0]), std::get<1>(data[0]));
            objectsids[0] = result;
            return objectsids;
        }

        std::vector<int> rearranged = rearrangeInput (pose, data);
        std::vector<std::tuple<int, Eigen::Vector3d> > arrData;

//        std::cout << "REARRANGE: ";
        for (int i = 0; i < rearranged.size(); i++) {
            arrData.push_back (data[rearranged[i]]);
//            std::cout << rearranged[i] << " ";
        }
//        std::cout << std::endl;

        // container to store path candidates
        // and distance values
        std::vector<std::vector<int> > candidates;

        // prepare input and data (map)
        std::vector<Eigen::Vector3d> measurements;
        std::vector<int> classids;
        std::vector<double> dz; // distance between measurements
        for (int i = 0; i < arrData.size(); i++) {
            measurements.push_back (std::get<1>(arrData[i]));
            classids.push_back (std::get<0>(arrData[i]));

            if (i < arrData.size() - 1) {
                double d = calculateDistance (std::get<1>(arrData[i]), std::get<1>(arrData[i+1]));
                dz.push_back (d);
            }
        }

        // fill first candidates with objects from _objectClassMap[classids[0]]
        // if _objectClassMap[classids[0]] is empty, put -1
        if (_graph->_objectClassMap[classids[0]].empty()) {
            std::vector<int> _candidates;
            _candidates.push_back (-1);
            candidates.push_back (_candidates);
        } else { // for 1st node, use nearest neighbor
//            int r = associate (pose, classids[0], measurements[0]);
//            std::vector<int> _candidates;
//            _candidates.push_back (r);
//            candidates.push_back (_candidates);

            for (auto it = _graph->_objectClassMap[classids[0]].begin(); 
                    it != _graph->_objectClassMap[classids[0]].end();
                   it ++) {
                std::vector<int> _candidates;
                _candidates.push_back (*it);
                candidates.push_back (_candidates);
            } 
        }

        std::vector<double> dcandidates (candidates.size(), 0.0); 
//        std::cout << "CANDIDATES SIZE: " << candidates.size() << std::endl;

//        if (candidates.size() == 1) std::cout << "IT IS: " << candidates[0][0] << std::endl;

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
//                std::cout << "FROM: " << candidates[j].back();
                for (auto jt = to.begin(); jt != to.end(); jt++) {

                    int id = -1;
                    Eigen::Vector3d o2;
                    if (*jt != -1)  {
                        auto kt = std::find (candidates[j].begin(), candidates[j].end(), *jt);
                        if (kt != candidates[j].end()) {
                            if (std::next(jt) != to.end()) continue;
                            else {
                                SE2 p; p.fromVector(pose->_pose);
                                SE2 m; m.fromVector(measurements[i]);
                                o2 = (p*m).toVector();
                                id = -1;
                            }
                        } else {
                            o2 = _graph->_objectMap[*jt]->_pose;
                            id = *jt;
                        }
                    } else {
                        SE2 p; p.fromVector(pose->_pose);
                        SE2 m; m.fromVector(measurements[i]);
                        o2 = (p*m).toVector();
                        id = -1;
                    }
                    
//                    std::cout << " TO: " << *jt;

                    double d = calculateDistance (o1, o2);
//                    std::cout << " --> D = " << d;

                    if (d == 0.0) continue; // if o1 = o2, discard

                    double diff = std::abs (d - dz[i-1]);
//                    std::cout << " || DIFF = " << diff;

                    if (diff < shortest) {
                        shortest = diff;
                        nextnode = id;
                    }
                }
//                std::cout << std::endl;

//                std::cout << "******* " << shortest << std::endl;
                if (shortest > 0.5) {
                    candidates[j].push_back (-1);
                } else {
                    candidates[j].push_back (nextnode);
                }

                dcandidates [j] += shortest; 
            }
        }

        double shortest_candidate = std::numeric_limits<double>::max();
        for (int i = 0; i < candidates.size(); i++){
            if (dcandidates[i] < shortest_candidate) {
                shortest_candidate = dcandidates[i];
                objectsids = candidates[i];
            }
        }

        std::vector<int> out (objectsids.size(), -1);
        for (int i = 0; i < objectsids.size(); i++) {
            out[rearranged[i]] = objectsids[i];
        }

//        std::cout << "OUT: ";
//        for (int i = 0; i < out.size(); i++) {
//            std::cout << out[i] << " ";
//        }
//        std::cout << std::endl;

        return out;
    }

    int DataAssociation::associate (Pose::Ptr pose, int classid, Eigen::Vector3d measurement) {
        
        std::set<int> obj = _graph->_objectClassMap[classid];

        if (obj.empty()) return -1;

        SE2 p; p.fromVector (pose->_pose); // pose
        SE2 m; m.fromVector (measurement); // measurement
        Eigen::Vector3d detected_obj_pose  = (p*m).toVector();

        double nearest_distance = std::numeric_limits<double>::max();
        int nearest_obj = -1;
        for (auto it = obj.begin(); it != obj.end(); it++) {

            double d = calculateDistance (detected_obj_pose, (_graph->_objectMap[*it])->_pose);

            if (d < nearest_distance) {
                nearest_distance = d;
                nearest_obj = (_graph->_objectMap[*it])->_id;
            }
        }

        if (nearest_distance < 0.5)
            return nearest_obj;
        else
            return -1;
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

    std::vector<int> DataAssociation::rearrangeInput (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data)
    {
        int size = data.size();
        std::vector<int> out (size, -1);

        std::map<int, double> arr;

        for (int i = 0; i < size; i++) {

            Eigen::Vector3d measurement = std::get<1>(data[i]);
            SE2 p; p.fromVector (pose->_pose);
            SE2 m; m.fromVector (measurement);
            Eigen::Vector3d o = (p*m).toVector();

            double shortest = std::numeric_limits<double>::max();

            // this prevent two class with no objects
            // having the same shortest values
            if (_graph->_objectClassMap[std::get<0>(data[i])].empty()) {
                shortest = 1000 + i * 1000; // random number
            } else {
                for (auto   it = _graph->_objectClassMap[std::get<0>(data[i])].begin();
                            it != _graph->_objectClassMap[std::get<0>(data[i])].end();
                            it++) {

                    double d = calculateDistance (o, _graph->_objectMap[*it]->_pose);

                    if (d < shortest) {
                        shortest = d;
                    }
                }
            }

            arr[i] = shortest;
        }

        std::vector<double> sorted;
        for (auto it = arr.begin(); it != arr.end(); it++) {
            sorted.push_back (it->second);
        }

        std::sort (sorted.begin(), sorted.end()); 

        for (int i = 0; i < sorted.size(); i++) {
            for (auto it = arr.begin(); it != arr.end(); it++) {
                if (sorted[i] == it->second)
                    out[i] = it->first;
            }
        } 

        return out;
    }

    std::vector<int> DataAssociation::rearrangeInput2 (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data)
    {
        int size = data.size();
        std::vector<int> out (size, -1);

        for (int i = 0; i < size; i++) {
            int size = _graph->_objectClassMap[std::get<0>(data[i])].size();
        }
    }

}
