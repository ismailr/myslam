#include <limits>
#include <algorithm>

#include "myslam_system/data_association.h"

#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"

namespace MYSLAM {

    DataAssociation::DataAssociation(Graph& graph)
        :_graph (&graph)
    {

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

        if (nearest_distance < 0.2)
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

        // this prevent first two nodes to have the same classids
        // as it has a potential to make a wrong associations
        if (std::get<0>(data[out[0]]) == std::get<0>(data[out[1]])) {
            int tmp = out[1];
            for (int i = 2; i < out.size(); i++) {
                if (std::get<0>(data[out[i]]) != std::get<0>(data[out[0]])) {
                    out [1] = out [i];
                    out[i] = tmp;
                    break;
                }
            }
        }

        return out;
    }

    std::vector<int> DataAssociation::associate (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data) 
    {
        int N = data.size();

        std::vector<int> objectsids (N, -1);

        // handle none and single object detection specially
        if (N == 0) return objectsids;
        else if (N == 1) {
            int result = associate (pose, std::get<0>(data[0]), std::get<1>(data[0]));
            objectsids[0] = result;
            return objectsids;
        }

        std::vector<int> rearranged = rearrangeInput (pose, data);
        std::vector<std::tuple<int, Eigen::Vector3d> > arrData;

        for (int i = 0; i < rearranged.size(); i++) {
            arrData.push_back (data[rearranged[i]]);
        }

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

        // first two nodes, find exhaustively
        std::set<int> node1 = _graph->_objectClassMap[classids[0]];
        std::set<int> node2 = _graph->_objectClassMap[classids[1]];

        if (node1.empty() && node2.empty()) {
            objectsids[0] = -1; 
            objectsids[1] = -1;
        } else {
            if (node1.empty()) node1.insert (-1); 
            if (node2.empty()) node2.insert (-1); 

            // measurements
            std::vector<Eigen::Vector3d> m; 
            for (int i = 0; i < 2; i++) {
                SE2 p; p.fromVector(pose->_pose);
                SE2 m_; m_.fromVector(measurements[i]);
                m.push_back((p*m_).toVector());
            }

            double shortest = std::numeric_limits<double>::max();
            double shortest_ = std::numeric_limits<double>::max(); // with one or both node -1
            int n1, n2; // current best pair
            int n1_, n2_; // current best pair with one or both node -1

            for (auto it = node1.begin(); it != node1.end(); it++) {
                Eigen::Vector3d o1;
                if (*it == -1) o1 = m[0];
                else o1 = _graph->_objectMap[*it]->_pose;

                for (auto jt = node2.begin(); jt != node2.end(); jt++) {
                    Eigen::Vector3d o2;
                    if (*jt == -1) o2 = m[1];
                    else o2 = _graph->_objectMap[*jt]->_pose;

                    double d = calculateDistance (o1, o2);
                    double diff = std::abs (d - dz[0]);

                    if (*it != -1 || *jt != -1) {
                        if (diff < shortest) {
                            shortest = diff;
                            n1 = *it;
                            n2 = *jt;
                        }
                    } else if (*it != -1 && *jt != -1) {
                        if (diff < shortest_) {
                            shortest_ = diff;
                            n1_ = *it;
                            n2_ = *jt;
                        }
                    }
                }

                if (shortest < 0.05) {
                    objectsids[0] = n1;
                    objectsids[1] = n2;
                } else if (shortest_ < 0.05) {
                    objectsids[0] = n1_;
                    objectsids[1] = n2_;
                } else {
                    objectsids[0] = -1;
                    objectsids[1] = -1;
                } 
            }
        }

        // start next nodes
        for (int i = 2; i < N; i++) {

            std::set<int> nodei = _graph->_objectClassMap[classids[i]];

            // remove object already in the objectsids
            for (auto it = nodei.begin(); it != nodei.end(); it++) {
                auto kt = std::find (objectsids.begin(), objectsids.end(), *it);
                if (kt != objectsids.end())
                    nodei.erase (*it);
            }

            if (nodei.empty()) {
                nodei.insert (-1);
                objectsids[i] = -1;
                continue;
            } 

            // previous node
            Eigen::Vector3d oh;
            if (objectsids[i-1] == -1) {
                SE2 p; p.fromVector(pose->_pose);
                SE2 m; m.fromVector(measurements[i-1]);
                oh = (p*m).toVector();
            } else {
                oh = _graph->_objectMap[objectsids[i-1]]->_pose;
            }

            double shortest = std::numeric_limits<double>::max();
            int ni;

            for (auto it = nodei.begin(); it != nodei.end(); ++it) {

                Eigen::Vector3d oi = _graph->_objectMap[*it]->_pose;

                double d = calculateDistance (oh, oi);
                double diff = std::abs (d - dz[i-1]);

                if (diff < shortest) {
                    shortest = diff;
                    ni = *it;
                } 
            }

            if (shortest < 0.05)
                objectsids[i] = ni;
            else
                objectsids[i] = -1;
        }

        std::vector<int> out (objectsids.size(), -1);
        for (int i = 0; i < objectsids.size(); i++) {
            out[rearranged[i]] = objectsids[i];
        }

        return out;
    }
}
