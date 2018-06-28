#ifndef _WALL_H_
#define _WALL_H_

#include <g2o/types/slam2d/vertex_point_xy.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace g2o;

namespace MYSLAM {
    class Line
    {
        public:
            Line(); 
            
            // parameters
            Eigen::Vector2d mc; // gradien intercept
            Eigen::Vector2d xx; // point perpendicular to origin
            Eigen::Vector2d rt; // rho and theta
            Eigen::Vector2d p; // endpoint
            Eigen::Vector2d q; // endpoint

            double length;
            double p2xx, q2xx;
            Eigen::Vector3d ppos, qpos;

            void calcXxFromMc();
            void calcXxFromRt();
            void calcRtFromMc();
            void calcRtFromXx();
            void calcMcFromXx();
            void calcMcFromRt();
            void calcPq();
            void calcSegment();
    };

    class Wall
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Wall> Ptr;

            Wall();
            void initParams ();
            void updateParams ();
            void updateSegment (Eigen::Vector2d p, Eigen::Vector2d q);

            unsigned long int _id;
            Line _line;
            pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
            Eigen::Matrix2d cov;
            std::set<int> _seenBy;
            bool _active;

    };

    class WallVertex : public VertexPointXY
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            WallVertex();
    };
}

#endif
