#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <iostream>
#include <string>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

using namespace g2o;

namespace MYSLAM {
    class Object {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Object> Ptr;

        Object ();

        unsigned long int _id;
        int _classid;
        Eigen::Vector3d _pose;
        bool _active;
	    std::string _type;

        std::set<int> _seenBy;
        int conf;

        private:
    };

    class ObjectXYZ {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<ObjectXYZ> Ptr;

            ObjectXYZ();

            unsigned long int _id;
            int _classid;
            Eigen::Vector3d _pose;
            bool _active;
            std::string _type;

            std::set<int> _seenBy;
            int _conf;

            private:
    };

    class ObjectVertex : public VertexSE2
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            ObjectVertex();
    };

    class ObjectXYZVertex : public VertexPointXYZ
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            ObjectXYZVertex();
    };
}

#endif
