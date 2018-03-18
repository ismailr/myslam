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
//       cov = Eigen::Matrix2d::Identity() * 99;
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
