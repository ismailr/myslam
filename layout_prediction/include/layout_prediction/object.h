#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <iostream>


#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include "se2.h"

using namespace g2o;

namespace MYSLAM {
    class Object {
        public:
            typedef std::shared_ptr<Object> Ptr;
            Object ();

            enum {
                MEJA,
                KURSI,
                PINTU,
                LEMARI,
                KOMPUTER };

            unsigned long int _id;
            Eigen::Vector3d _pose;
            Eigen::Matrix<double,3,3> objectCovarianceMatrix;

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
