#ifndef _POSE_H_
#define _POSE_H_

#include <memory> // shared_ptr
#include <set>

#include <Eigen/Geometry>

namespace MYSLAM {
    class Pose3D
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Pose3D> Ptr;
            Pose3D();

            unsigned long int _id;
            Eigen::Affine3d _pose;
            bool _active;

            std::set<int> _detectedWalls;
            std::set<int> _detectedObjects;
    };

    class Pose2D
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Pose2D> Ptr;
            Pose2D();

            unsigned long int _id;
            Eigen::Vector3d _pose;

            std::set<int> _detectedWalls;
            std::set<int> _detectedObjects;
    };
}
#endif
