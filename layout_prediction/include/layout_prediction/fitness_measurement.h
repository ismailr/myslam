#ifndef _FITNESS_MEASUREMENT_H_
#define _FINTESS_MEASUREMENT_H_

#include "g2o/config.h"
#include "layout_prediction/wall.h"
#include "g2o/core/base_unary_edge.h"

class FitnessMeasurement : BaseUnaryEdge <1, double, Wall2>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<FitnessMeasurement> Ptr;
    typedef std::shared_ptr<const FitnessMeasurement> ConstPtr;

    FitnessMeasurement();

    void computeError()
    {
        const Wall2* w = static_cast<const Wall2*>(_vertices[0]);

    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setMeasurement(const double m){
        _measurement = m;
    }

    virtual bool setMeasurementFromState() {
        return true;
    }
};

#endif
