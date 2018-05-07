#ifndef _DATA_ASSOCIATION_H_
#define _DATA_ASSOCIATION_H_

#include "layout_prediction/graph.h"

namespace MYSLAM {
    class DataAssociation {
        public:
            DataAssociation (Graph&);

//            void associate (std::vector<Eigen::Vector3d>, std::Vector<int>);
            void associate (Pose::Ptr, std::vector<std::tuple<int, Eigen::Vector3d> >); 

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
