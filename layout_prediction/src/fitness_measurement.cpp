#include "layout_prediction/fitness_measurement.h"

FitnessMeasurement::FitnessMeasurement()
    : BaseUnaryEdge <1, double, Wall2>()
{
    _information.setIdentity();
    _error.setZero();
}

bool FitnessMeasurement::read(std::istream& is)
{
    double d;
    is >> d;
    setMeasurement(d);
    return true;
}

bool FitnessMeasurement::write(std::ostream& os) const
{
    os << _measurement << " ";
    return os.good();
}
