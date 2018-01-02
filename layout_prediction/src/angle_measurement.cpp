#include "layout_prediction/angle_measurement.h"

namespace MYSLAM {
    AngleMeasurement::AngleMeasurement()
        : BaseBinaryEdge <1, double, WallVertex, WallVertex>()
    {
        _information.setIdentity();
        _error.setZero();
    }

        bool AngleMeasurement::read(std::istream& is)
        {
            double d;
            is >> d;
            setMeasurement(d);
            return true;
        }

        bool AngleMeasurement::write(std::ostream& os) const
        {
            os << _measurement << " ";
            return os.good();
        }


    //#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
    //  void EdgeLine2D::linearizeOplus()
    //  {
    //    _jacobianOplusXi=-Matrix2d::Identity();
    //    _jacobianOplusXj= Matrix2d::Identity();
    //  }
}
