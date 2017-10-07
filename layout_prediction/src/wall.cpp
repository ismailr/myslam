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

#include <typeinfo>
#include <algorithm>
#include <math.h>
#include <assert.h>

#include "g2o/stuff/macros.h"

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"

Wall::Wall() :
BaseVertex<2, Line2D>()
{
    _estimate.setZero();
    _p.setZero();
    _q.setZero();
    _pq.setZero();
    _fitness = 0.0;
    p1Id=p2Id=-1;
    _rhoGlobal = 0.0;
    _thetaGlobal = 0.0;
}

Wall::Wall(double rho, double theta) :
BaseVertex<2, Line2D>()
{
    setTheta (theta);
    setRho (rho);
    _p.setZero();
    _q.setZero();
    _pq(0) = rho * cos(theta);
    _pq(1) = rho * sin(theta);
    _fitness = 0.0;
    _rhoGlobal = 0.0;
    _thetaGlobal = 0.0;
}

Wall::Wall(double rho, double theta, Eigen::Vector2d p, Eigen::Vector2d q) :
BaseVertex<2, Line2D>()
{
    setTheta (theta);
    setRho (rho);
    _p = p;
    _q = q;
    _pq(0) = abs (_p(0) - _q(0))/2 + std::min (_p(0),_q(0));
    _pq(1) = abs (_p(1) - _q(1))/2 + std::min (_p(1),_q(1));
    _fitness = 0.0;
    _rhoGlobal = 0.0;
    _thetaGlobal = 0.0;
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

void Wall::setObserverPose (int poseId)
{
    _observerPoses.push_back (poseId);
}

Wall2::Wall2 () : VertexLine2D(){}
//Wall2::Wall2 (double rho, double theta)
//    :_fitness (0.0)
//{
//    setRho (rho);
//    setTheta (theta);
//    _gradient = -1/tan(theta);
//    _intercept = rho * sin (theta);
//}

Wall2::Wall2 (double gradient, double intercept)
    :_gradient (gradient), _intercept (intercept), _fitness (0.0)
{
    // rho = |intercept|/(gradien^2) + 1)^(0.5)
    // theta = arctan (- 1/gradien)
    double rho = std::abs (_intercept) /sqrt (pow (_gradient, 2) + 1);
    double theta = atan(-1/_gradient) * 180/M_PI;
    setRho (rho);
    setTheta (theta);
}

void Wall2::set_inliers (Inliers inliers)
{
    _inliers = inliers;
    calculate_fitness ();
    calculate_edge_points ();
}

void Wall2::calculate_fitness ()
{
    if (_inliers.empty())
    {
        _fitness = 1000;
        return;
    }

    /* *************************
     * Distance from a point to a line y = mx + d
     * line eq: ax + by + c = 0
     *          a = m, b = -1, c = d
     * 
     *  distance from (x0,y0) to ax + by + c is
     *
     *  d = |ax0 + by0 + c|
     *      ---------------
     *      sqrt (a^2 + b^2)
     *
     *    = |mx0 - y0 + d|
     *      ---------------
     *      sqrt (m^2 + 1)
     *
     * ************************** */

    double d = 0.0;
    for (Inliers::const_iterator it = _inliers.begin(); 
            it != _inliers.end(); it++)
    {
        double x0 = (*it)[0];
        double y0 = (*it)[1];
        double z0 = (*it)[2];

        _fitness += std::abs (_gradient * x0 - y0 + _intercept)/sqrt (_gradient * _gradient + 1);
    }
}

void Wall2::calculate_edge_points ()
{
    assert (!_inliers.empty());

    Eigen::Vector3d _p (_inliers.front());
    Eigen::Vector3d _q (_inliers.back());

    double py = _gradient * _p[0] + _intercept;
    double qy = _gradient * _q[0] + _intercept;

    Eigen::Vector2d p (_p[0], py);
    Eigen::Vector2d q (_q[0], qy);

    _endPoints = std::make_tuple (p,q);
}
