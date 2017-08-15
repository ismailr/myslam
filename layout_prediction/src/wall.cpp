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

#include "layout_prediction/wall.h"
#include <typeinfo>
#include "g2o/stuff/macros.h"

Wall::Wall() :
BaseVertex<2, Line2D>()
{
    _estimate.setZero();
    _p.setZero();
    _q.setZero();
    _fitness = 0.0;
    p1Id=p2Id=-1;
}

Wall::Wall(double rho, double theta) :
BaseVertex<2, Line2D>()
{
    setTheta (theta);
    setRho (rho);
    _p.setZero();
    _q.setZero();
    _fitness = 0.0;
}

Wall::Wall(double rho, double theta, Eigen::Vector2d p, Eigen::Vector2d q) :
BaseVertex<2, Line2D>()
{
    setTheta (theta);
    setRho (rho);
    _p = p;
    _q = q;
    _fitness = 0.0;
}

bool Wall::read(std::istream& is)
{
    is >> _estimate[0] >> _estimate[1] >> p1Id >> p2Id;
    return true;
}

bool Wall::write(std::ostream& os) const
{
    os << estimate()(0) << " " << estimate()(1) << " " << p1Id << " " << p2Id;
    return os.good();
}

double Wall::getFitness ()
{
    return _fitness;
}

void Wall::setFitness (double fitness)
{
    _fitness = fitness;
}
