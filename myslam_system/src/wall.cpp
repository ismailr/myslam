#include <math.h>
#include <fstream>

#include "myslam_system/wall.h"
#include "myslam_system/helpers.h"

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


    Wall::Wall() : _active (false) {
       _id = Generator::id++;
//       cov = Eigen::Matrix2d::Identity() * 99;
    };

    void Wall::updateParams(){
        _line.calcSegment();
        _line.calcMcFromXx();
        _line.calcPq();
    };

    void Wall::initParams(){
        _line.calcMcFromXx();
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
