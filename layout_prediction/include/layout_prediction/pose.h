#ifndef _POSE_H_
#define _POSE_H_

#include <memory>

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/se2.h>

using namespace g2o;

namespace MYSLAM {
    class Pose
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Pose> Ptr;
            Pose();

            unsigned long int _id;
            Eigen::Vector3d _pose;
            bool _active;

            std::set<int> _detectedWalls;
            std::set<int> _detectedObjects;

    };

    class PoseVertex: public VertexSE2
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            PoseVertex();
    };
}
#endif
