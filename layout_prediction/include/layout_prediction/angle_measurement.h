#ifndef _ANGLE_MEASUREMENT_H_
#define _ANGLE_MEASUREMENT_H_

#include <algorithm>

#include "g2o/config.h"
#include "layout_prediction/wall.h"
#include "g2o/core/base_binary_edge.h"

class AngleMeasurement : public BaseBinaryEdge<1, double, Wall3, Wall3>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<AngleMeasurement> Ptr;
    typedef std::shared_ptr<const AngleMeasurement> ConstPtr;

    AngleMeasurement();

    void computeError()
    {
        const Wall3* w1 = static_cast<const Wall3*>(_vertices[0]);
        const Wall3* w2 = static_cast<const Wall3*>(_vertices[1]);

        double w1xx = w1->estimate().x();
        double w1xy = w1->estimate().y();
        double w2xx = w2->estimate().x();
        double w2xy = w2->estimate().y();

        double theta1 = atan2 (w1xy, w1xx);
        double theta2 = atan2 (w2xy, w2xx);

        double delta_theta = std::abs (theta2 - theta1);
        delta_theta < 45 * M_PI/180 ? _error(0,0) = theta2 - 0 : _error(0,0) = theta2 - (M_PI/2) ;
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setMeasurement(const double m){
        _measurement = m;
    }

//    virtual bool setMeasurementFromState() {
//        const Wall3* w1 = static_cast<const Wall3*>(_vertices[0]);
//        const Wall3* w2 = static_cast<const Wall3*>(_vertices[1]);
//
//        _measurement = std::abs (w2->theta() - w1->theta());
//        return true;
//    }


//    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 0.;}
//#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
//    virtual void linearizeOplus();
//#endif

};

#endif
