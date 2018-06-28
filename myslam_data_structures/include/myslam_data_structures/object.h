#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <memory>
#include <set>

#include <Eigen/Geometry>

namespace MYSLAM {
    class Object3D {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Object3D> Ptr;

            Object3D ();

            unsigned long int _id;
            int _classid;
            Eigen::Affine3d _pose;
            bool _active;

            std::set<int> _seenBy;
    };

    class Object2D {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Object2D> Ptr;

            Object2D ();

            unsigned long int _id;
            int _classid;
            Eigen::Vector3d _pose;
            bool _active;

            std::set<int> _seenBy;
    };
}

#endif
