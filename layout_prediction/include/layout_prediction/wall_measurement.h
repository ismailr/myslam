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

#ifndef _WALL_MEASUREMENT_H_
#define _WALL_MEASUREMENT_H_

#include <memory>
#include <fstream>
#include <limits>

#include "g2o/config.h"
#include "pose.h"
#include "wall.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/stuff/misc.h"
#include "edge_se2_line2d.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    class WallMeasurement : public EdgeSE2PointXY
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            WallMeasurement();

//            void computeError()
//            {
//                const PoseVertex* v1 = static_cast<const PoseVertex*>(_vertices[0]);
//                const WallVertex* l2 = static_cast<const WallVertex*>(_vertices[1]);
//
//                double xx = l2->estimate().x();
//                double xy = l2->estimate().y();
//                double m, c;
//                m = -xx/xy;
//                c = (xx*xx + xy*xy)/xy;
//
//                double x = v1->estimate().translation().x();
//                double y = v1->estimate().translation().y();
//                double p = v1->estimate().rotation().angle();
//
//                double m_, c_;
//                m_ = (-sin(p) + m * cos(p))/(cos(p) + m * sin(p));
//                c_ = (c - y + m * x)/(cos(p) + m * sin(p));
//
//                double xx_, xy_;
//                xx_ = (-m_*c_)/(m_*m_+1);
//                xy_ = c_/(m_*m_+1);
//
//                Eigen::Vector2d _prediction (xx_, xy_);
//                _error = _prediction - _measurement;
//
//                std::ofstream f;
//                f.open ("/home/ism/tmp/error1.dat", std::ios::out|std::ios::app);
//                f << "x = " << x << " y = " << y << " p = " << p << std::endl; 
//                f << "m = " << m << " c = " << c << std::endl; 
//                f << "m' = " << m_ << " c' = " << c_ << std::endl; 
//                f << "xx' = " << xx_ << " xy' = " << xy_ << std::endl;
//                f << "xx = " << l2->estimate().x() << " xy = " << l2->estimate().y() << std::endl;
//                f << "PREDICTION: " << _prediction.transpose() << std::endl;
//                f << "MEASUREMENT: " << _measurement.transpose() << std::endl;
//                f << "ERROR: " << _error.transpose() << std::endl;
//                f << std::endl;
//                f.close();
//            }
    };
}
#endif
