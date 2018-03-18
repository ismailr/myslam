// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (C) 2017 Ismail
// All rights reserved.

#ifndef _WALL_H_
#define _WALL_H_

#include <string>

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/stuff/misc.h>
#include <g2o/types/slam2d/vertex_point_xy.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vertex_line2d.h"

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
            typedef std::shared_ptr<Wall> Ptr;
            Wall();
            void initParams ();
            void updateParams ();
            void updateSegment (Eigen::Vector2d p, Eigen::Vector2d q);

            unsigned long int _id;
            Line _line;
            cv::Ptr<cv::Mat> _img;
            pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
            Eigen::Matrix2d cov;
            std::set<int> _seenBy;

    };

    class WallVertex : public VertexPointXY
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            WallVertex();
    };
}

#endif
