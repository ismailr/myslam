#ifndef _ANGLE_MEASUREMENT_H_
#define _ANGLE_MEASUREMENT_H_

#include <algorithm>

#include "g2o/config.h"
#include "layout_prediction/wall.h"
#include "g2o/core/base_binary_edge.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    class AngleMeasurement : public BaseBinaryEdge<1, double, WallVertex, WallVertex>
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<AngleMeasurement> Ptr;
        typedef std::shared_ptr<const AngleMeasurement> ConstPtr;

        AngleMeasurement();

        void computeError()
        {
            const WallVertex* w1 = static_cast<const WallVertex*>(_vertices[0]);
            const WallVertex* w2 = static_cast<const WallVertex*>(_vertices[1]);

            double w1xx = w1->estimate().x();
            double w1xy = w1->estimate().y();
            double w2xx = w2->estimate().x();
            double w2xy = w2->estimate().y();

            double theta1 = atan2 (w1xy, w1xx);
            double theta2 = atan2 (w2xy, w2xx);

            double delta_theta = normalize_angle (std::abs (theta2 - theta1));
            delta_theta < 45 * M_PI/180 ? _error(0,0) = theta2 - 0 : _error(0,0) = theta2 - (M_PI/2) ;

            double b0 = 0.0;
            double b1 = 30.0 * M_PI/180.0;
            double b2 = 60.0 * M_PI/180.0;
            double b3 = 90.0 * M_PI/180.0;
            double b4 = 120.0 * M_PI/180.0;
            double b5 = 150.0 * M_PI/180.0;
            double b6 = 180.0 * M_PI/180.0;
            double b7 = 210.0 * M_PI/180.0;
            double b8 = 240.0 * M_PI/180.0;
            double b9 = 270.0 * M_PI/180.0;
            double b10 = 300.0 * M_PI/180.0;
            double b11 = 330.0 * M_PI/180.0;
            double b12 = 360.0 * M_PI/180.0;

            if (delta_theta >= b0 && delta_theta < b1)
                _error(0,0) = std::abs(theta2 - 0.0);
            else if (delta_theta >= b1 && delta_theta < b2)
                _error(0,0) = std::abs(theta2 - 45.0 * M_PI/180);
            else if (delta_theta >= b2 && delta_theta < b4)
                _error(0,0) = std::abs(theta2 - 90.0 * M_PI/180);
            else if (delta_theta >= b4 && delta_theta < b5)
                _error(0,0) = std::abs(theta2 - 135.0 * M_PI/180);
            else if (delta_theta >= b5 && delta_theta < b7)
                _error(0,0) = std::abs(theta2 - 180.0 * M_PI/180);
            else if (delta_theta >= b7 && delta_theta < b8)
                _error(0,0) = std::abs(theta2 - 225.0 * M_PI/180);
            else if (delta_theta >= b8 && delta_theta < b10)
                _error(0,0) = std::abs(theta2 - 270.0 * M_PI/180);
            else if (delta_theta >= b10 && delta_theta < b11)
                _error(0,0) = std::abs(theta2 - 315.0 * M_PI/180);
            else if (delta_theta >= b11 && delta_theta <= b12)
                _error(0,0) = std::abs(theta2 - 360.0 * M_PI/180);
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

}

#endif
