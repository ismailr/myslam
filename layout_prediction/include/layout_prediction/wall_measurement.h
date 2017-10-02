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

#include "g2o/config.h"
#include "pose.h"
#include "wall.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/stuff/misc.h"
#include "edge_se2_line2d.h"

  class WallMeasurement: public BaseBinaryEdge<2, Line2D, Pose, Wall>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      WallMeasurement();

      typedef std::shared_ptr<WallMeasurement> Ptr;
      typedef std::shared_ptr<const WallMeasurement> ConstPtr;

      void computeError()
      {
        const Pose* v1 = static_cast<const Pose*>(_vertices[0]);
        const Wall* l2 = static_cast<const Wall*>(_vertices[1]);
        Vector2D prediction=l2->estimate();
        SE2 iT=v1->estimate().inverse();
        prediction[0] += iT.rotation().angle();
        prediction[0] = normalize_theta(prediction[0]);
        Vector2D n(cos(prediction[0]), sin(prediction[0]));
        prediction[1] += n.dot(iT.translation());
        _error =  prediction - _measurement;
        _error [0] =  normalize_theta(_error[0]);
      }

      virtual bool setMeasurementData(const double* d){
        _measurement[0]=d[0];
        _measurement[1]=d[1];
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        d[0] = _measurement[0];
        d[1] = _measurement[1];
        return true;
      }

      virtual int measurementDimension() const {return 2;}

      virtual bool setMeasurementFromState(){
        const Pose* v1 = static_cast<const Pose*>(_vertices[0]);
        const Wall* l2 = static_cast<const Wall*>(_vertices[1]);
        Vector2D prediction=l2->estimate();
        SE2 iT=v1->estimate().inverse();
        prediction[0] += iT.rotation().angle();
        prediction[0] = normalize_theta(prediction[0]);
        Vector2D n(cos(prediction[0]), sin(prediction[0]));
        prediction[1] += n.dot(iT.translation());
        _measurement = prediction;
        return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);}
// #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES 
//       virtual void linearizeOplus(); 
// #endif 
  };

/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Line2DWriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeSE2Line2DWriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Line2DDrawAction: public DrawAction{ */
/*   public: */
/*     EdgeSE2Line2DDrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

class WallMeasurement2 : public EdgeSE2Line2D
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        WallMeasurement2();

    private:

};

#endif
