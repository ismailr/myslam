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

#ifndef G2O_TYPES_SLAM2D_ONLINE_H
#define G2O_TYPES_SLAM2D_ONLINE_H

#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include <iostream>

using namespace g2o;

namespace MYSLAM {
  class OnlineVertexSE2 : public VertexSE2
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      OnlineVertexSE2() : VertexSE2() {}

      virtual void oplusImpl(const double* update)
      {
        VertexSE2::oplusImpl(update);
        updatedEstimate = _estimate;
      }

      void oplusUpdatedEstimate(double* update)
      {
        Eigen::Vector3d p=_estimate.toVector();
        p+=Eigen::Map<Eigen::Vector3d>(update);
        p[2]=normalize_theta(p[2]);
        updatedEstimate.fromVector(p);
        //std::cerr << PVAR(updatedEstimate.toVector()) << " " << PVAR(_estimate.toVector()) << std::endl;
      }

      VertexSE2::EstimateType updatedEstimate;
  };

    class OnlineEdgeSE2 : public EdgeSE2
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        OnlineEdgeSE2() : EdgeSE2() {}

        void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /* to */)
        {
            OnlineVertexSE2* fromEdge = static_cast<OnlineVertexSE2*>(_vertices[0]);
            OnlineVertexSE2* toEdge   = static_cast<OnlineVertexSE2*>(_vertices[1]);
            if (from.count(fromEdge) > 0) {
              toEdge->updatedEstimate = fromEdge->updatedEstimate * _measurement;
              toEdge->setEstimate(toEdge->updatedEstimate);
            } else {
              fromEdge->updatedEstimate = toEdge->updatedEstimate * _inverseMeasurement;
              fromEdge->setEstimate(fromEdge->updatedEstimate);
            }
        }

        double chi2() const
        {
            const OnlineVertexSE2* v1 = static_cast<const OnlineVertexSE2*>(_vertices[0]);
            const OnlineVertexSE2* v2 = static_cast<const OnlineVertexSE2*>(_vertices[1]);
            SE2 delta = _inverseMeasurement * (v1->updatedEstimate.inverse()*v2->updatedEstimate);
            Eigen::Vector3d error = delta.toVector();
            return error.dot(information() * error);
        }
    };

    class OnlineVertexPointXY : public VertexPointXY
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        OnlineVertexPointXY() : VertexPointXY() {}

        virtual void oplusImpl(const double* update) {
            VertexPointXY::oplusImpl(update);
            updatedEstimate = _estimate;
        }

        void oplusUpdatedEstimate(double* update) {
            updatedEstimate[0] += update[0];
            updatedEstimate[1] += update[1];
        }

        VertexPointXY::EstimateType updatedEstimate;
    };

    class OnlineEdgeSE2PointXY : public EdgeSE2PointXY
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        OnlineEdgeSE2PointXY() : EdgeSE2PointXY() {}

        void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /* to */)
        {
            OnlineVertexSE2* fromEdge = static_cast<OnlineVertexSE2*>(_vertices[0]);
            OnlineVertexPointXY* toEdge   = static_cast<OnlineVertexPointXY*>(_vertices[1]);

            double x = fromEdge->updatedEstimate.translation().x();
            double y = fromEdge->updatedEstimate.translation().y();
            double p = fromEdge->updatedEstimate.rotation().angle();
            double sinp = sin(p);
            double cosp = cos(p);

            double xx_ = _measurement[0];
            double xy_ = _measurement[1];

            double m_ = -xx_/xy_;
            double c_ = (xx_*xx_ + xy_*xy_)/xy_;

            double m, c;
            m = (sinp +  m_ * cosp)/(cosp - m_ * sinp);
            c = -m * x + y + c_/(cosp - m_ * sinp);

            double xx, xy;
            xx = -m*c/(m*m+1);
            xy = c/(m*m+1);

            toEdge->updatedEstimate[0] = xx;
            toEdge->updatedEstimate[1] = xy;
            toEdge->setEstimate(toEdge->updatedEstimate);
        }

        double chi2() const
        {
            const OnlineVertexSE2* v1 = static_cast<const OnlineVertexSE2*>(_vertices[0]);
            const OnlineVertexPointXY* l2 = static_cast<const OnlineVertexPointXY*>(_vertices[1]);

            double xx = l2->updatedEstimate[0];
            double xy = l2->updatedEstimate[1];
            double m, c;
            m = -xx/xy;
            c = (xx*xx + xy*xy)/xy;

            double x = v1->updatedEstimate.translation().x();
            double y = v1->updatedEstimate.translation().y();
            double p = v1->updatedEstimate.rotation().angle();

            double m_, c_;
            m_ = (-sin(p) + m * cos(p))/(cos(p) + m * sin(p));
            c_ = (c - y + m * x)/(cos(p) + m * sin(p));

            double xx_, xy_;
            xx_ = (-m_*c_)/(m_*m_+1);
            xy_ = c_/(m_*m_+1);

            Eigen::Vector2d error (xx_ - _measurement[0], xy_ - _measurement[1]);
            return error.dot(information() * error);
        }
    };
} // end namespace

#endif
