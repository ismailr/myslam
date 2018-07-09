#ifndef _DATA_ASSOCIATION_H_
#define _DATA_ASSOCIATION_H_

#include "layout_prediction/graph.h"

namespace MYSLAM {
    class DataAssociation {
        public:
            DataAssociation (Graph&);

            // args: robot's pose, vector<classid, measurements>
            // return: set of objects' ids
            std::vector<int> associate (Pose::Ptr, std::vector<std::tuple<int, Eigen::Vector3d> >); 
            int associate (Pose::Ptr, int, Eigen::Vector3d);
            std::vector<int> rearrangeInput (Pose::Ptr, std::vector<std::tuple<int, Eigen::Vector3d> >);

        private:
            Graph* _graph;

            double calculateDistance (Wall::Ptr, Wall::Ptr);
            double calculateDistance (Wall::Ptr, Object::Ptr);
            double calculateDistance (Object::Ptr, Object::Ptr);
            double calculateDistance (Eigen::Vector3d, Eigen::Vector3d);
            double calculateTotalDistance (std::vector<int>);
            double calculateTotalDistance (std::vector<Eigen::Vector3d>);
            double calculateDiff (std::vector<int>, std::vector<int>);
    };
}

#endif
