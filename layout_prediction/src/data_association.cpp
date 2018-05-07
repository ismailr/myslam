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

//    void DataAssociation::associate (std::vector<Eigen::Vector3d> objects, std::vector<int> classids) {
    void DataAssociation::associate (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data) {

        std::vector<Eigen::Vector3d> measurements;
        std::vector<int> classids;

        for (int i = 0; i < data.size(); i++) {
            measurements.push_back (std::get<1>(data[i]));
            classids.push_back (std::get<0>(data[i]));
        }

        int numOfSeq = 1;
        for (int i = 0; i < classids.size(); i++) {

            // first, handle new detected object
            if (_graph->_objectClassMap[classids[i]].size() == 0) {
                Object::Ptr o (new Object);
                SE2 robotPose; robotPose.fromVector (pose->_pose);
                SE2 measurement; measurement.fromVector (measurements[i]);
                o->_pose = (robotPose * measurement).toVector();
                o->_classid = classids[i];
                _graph->insertNode (o);
            }

            numOfSeq *= _graph->_objectClassMap[classids[i]].size();
        }

        double distData = calculateTotalDistance (measurements);
        double minDelta = std::numeric_limits<double>::max();

        for (int i = 0; i < numOfSeq; i++) {

            std::vector<int> seq; // current sequence


            // brute force

            // calculate dist of current seq to data
            double distSeq = calculateTotalDistance (seq);
            double delta = std::abs (distData - distSeq);
            if (delta < minDelta) minDelta = delta;
        }
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
