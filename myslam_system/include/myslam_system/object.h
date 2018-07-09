#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <iostream>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/se2.h>

using namespace g2o;

namespace MYSLAM {
    class Object {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Object> Ptr;

            Object ();

            enum {
                MEJA,
                KURSI,
                PINTU,
                LEMARI,
                KOMPUTER };

            unsigned long int _id;
            int _classid;
            Eigen::Vector3d _pose;
            bool _active;

            std::set<int> _seenBy;

        private:
    };

    class ObjectVertex : public VertexSE2
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            ObjectVertex();
    };
}

#endif
