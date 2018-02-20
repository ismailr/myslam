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
#include <fstream>
#include <initializer_list>

#include "g2o/stuff/macros.h"

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/helpers.h"

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

Wall2::Wall2 () : VertexLine2D()
{
    nodetype = "WALL2";
}
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
    double theta = atan(-1/_gradient);
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
     G* line eq: ax + by + c = 0
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

//    double py = _gradient * _p[0] + _intercept;
//    double qy = _gradient * _q[0] + _intercept;

//    Eigen::Vector2d p (_p[0], py);
//    Eigen::Vector2d q (_q[0], qy);

//    std::ofstream wallfile;
//    wallfile.open ("/home/ism/tmp/wall.dat",std::ios::out|std::ios::app);
//    wallfile << p.transpose() << std::endl;
//    wallfile << q.transpose() << std::endl;
//    wallfile.close();
//
    Eigen::Vector2d p (_p[0], _p[1]);
    Eigen::Vector2d q (_q[0], _q[1]);

    _endPoints = std::make_tuple (p,q);
    calculate_center_point();
}

void Wall2::calculate_center_point()
{
    Eigen::Vector2d p = std::get<0>(_endPoints);
    Eigen::Vector2d q = std::get<1>(_endPoints);
    double cx, cy;
    double big, small;
    if (p(0) < q(0))
    {
        cx = ((q(0) - p(0))/2) + q(0);
        cy = ((q(1) - p(1))/2) + q(1);
    }
    else 
    {
        cx = ((p(0) - q(0))/2) + p(0);
        cy = ((p(1) - q(1))/2) + p(1);
    }

    _centerPoint[0] = cx;
    _centerPoint[1] = cy;
}

//void Wall2::calculate_gradient_intercept()
//{
//    _gradient = -1/tan (_estimate[0]);
//    _intercept = _gradient < 0  ? - _estimate[1] * sqrt(pow(_gradient,2) + 1)
//                                : _estimate[1] * sqrt(pow(_gradient,2) + 1);
//}

void Wall2::updateData()
{
    // update  gradient and intercept
    _gradient = -1/tan(_estimate[0]);
    _intercept = _estimate[1]/sin(_estimate[0]);


}

Wall3::Wall3 () : VertexPointXY()
{
    nodetype = "WALL3";
    _m = 0.0;
    _c = 0.0;
    _p(0) = _p(1) = _q(0) = _q(1) = 0.0;
}

void Wall3::updateIntermediateParams ()
{
    mcFromEstimate();
//    pqFromEstimate();
}

void Wall3::mcFromEstimate()
{
    double& x = _estimate[0];
    double& y = _estimate[1];

    if (y != 0)
    {
        _m = -x/y;
        _c = (x*x + y*y)/y;
    }
}

void Wall3::pqFromEstimate()
{
    double& x = _estimate[0];
    double& y = _estimate[1];
    Eigen::Vector2d r (x,y);

    double norm = sqrt (1 + _m*_m);
    Eigen::Vector2d unitm (1/norm,_m/norm);

    _p = r - _l * unitm;
    _q = r + _r * unitm;
}

void Wall3::setpq (Eigen::Vector2d p, Eigen::Vector2d q) 
{
    _p = q;
    _q = p;

    double& x = _estimate[0];
    double& y = _estimate[1];

    _l = sqrt ((x-_p.x())*(x-_p.x()) + (y-_p.y())*(y-_p.y()));
    _r = sqrt ((x-_q.x())*(x-_q.x()) + (y-_q.y())*(y-_q.y()));
}

namespace MYSLAM {
    Line::Line(){
        mc.setZero();
        xx.setZero();
        rt.setZero();
        p.setZero();
        q.setZero();
    };

    void Line::calcXxFromMc(){};
    void Line::calcXxFromRt(){};
    void Line::calcRtFromMc(){};
    void Line::calcRtFromXx(){};

    void Line::calcMcFromXx(){
        double& m = mc[0];
        double& c = mc[1];

        m = -xx[0]/xx[1];
        c = (xx[0]*xx[0] + xx[1]*xx[1])/xx[1];
    };

    void Line::calcMcFromRt(){};

    void Line::calcPq(){
        double& px = p[0];
        double& py = p[1];
        double& qx = q[0];
        double& qy = q[1];
        double& x = xx[0];
        double& y = xx[1];

        double& m = mc[0];
        double& c = mc[1];

        // line with gradient m through (0,0) is y = mx
        // vector in line (x, mx)
        // unit vector sqrt (x,mx)/(x*x + mx*mx)
        // for x = 1

        double normalizer = 1 + m*m;
        Eigen::Vector2d unit (1/normalizer, m/normalizer);

        Eigen::Vector3d x3d (x, y, 0);
        Eigen::Vector3d p3d (px, py, 0);
        Eigen::Vector3d q3d (qx, qy, 0);

        Eigen::Vector3d presult = x3d + p2xx * (x3d.cross(-ppos)).normalized();
        Eigen::Vector3d qresult = x3d + q2xx * (x3d.cross(-qpos)).normalized();

        p[0] = presult[0];
        p[1] = presult[1];
        q[0] = qresult[0];
        q[1] = qresult[1];
    };

    void Line::calcSegment() {
        double& px = p[0];
        double& py = p[1];
        double& qx = q[0];
        double& qy = q[1];
        double& x = xx[0];
        double& y = xx[1];

        length = sqrt ((px-qx)*(px-qx)+(py-qy)*(py-qy));

        Eigen::Vector3d x3d (x, y, 0);
        Eigen::Vector3d p3d (px, py, 0);
        Eigen::Vector3d q3d (qx, qy, 0);

        Eigen::Vector3d pd = p3d - x3d;
        Eigen::Vector3d qd = q3d - x3d;

        p2xx = pd.norm();
        q2xx = qd.norm();

        ppos = (x3d.cross(pd)).normalized();
        qpos = (x3d.cross(qd)).normalized();
    };


    Wall::Wall():_img(new cv::Mat()) {
       _id = Generator::id++;
       cov = Eigen::Matrix2d::Identity() * 99;
    };

    void Wall::updateParams(){
        _line.calcMcFromXx();
        _line.calcPq();
    };

    void Wall::initParams(){
        _line.calcSegment();
    };

    void Wall::updateSegment(Eigen::Vector2d p, Eigen::Vector2d q) {

        double _px = _line.p[0];
        double _py = _line.p[1];
        double _qx = _line.q[0];
        double _qy = _line.q[1];
        double px = p[0];
        double py = p[1];
        double qx = q[0];
        double qy = q[1];

        Eigen::Vector2d _l, _r;
        if (_px < _qx) {
            _l = _line.p;
            _r = _line.q;
        } else {
            _l = _line.q;
            _r = _line.p;
        }

        Eigen::Vector2d l, r;
        if (px < qx) {
            l = p;
            r = q;
        } else {
            l = q;
            r = p;
        }

        if (l[0] < _l[0]) {
            _line.p[0] = l[0];
            _line.p[1] = l[1];

        } else {
            _line.p[0] = _l[0];
            _line.p[1] = _l[1];
        }

        if (r[0] < _r[0]) {
            _line.q[0] = _r[0];
            _line.q[1] = _r[1];

        } else {
            _line.q[0] = r[0];
            _line.q[1] = r[1];
        }

        std::ofstream pqfile;
        pqfile.open ("/home/ism/tmp/pq.dat", std::ios::out | std::ios::app);
        pqfile  << _px << " " << _py << " " << _qx << " " << _qy << " "
                << p.transpose() << " " << q.transpose() << std::endl; 
        pqfile  << _l.transpose() << " " << _r.transpose() << " "
                << l.transpose() << " " << r.transpose() << std::endl;
        pqfile << _line.p.transpose() << " " << _line.q.transpose() << std::endl << std::endl;
        pqfile.close();
        
        _line.calcSegment();
    };

    WallVertex::WallVertex(){};

}
