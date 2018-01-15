#ifndef _POSE2D_H_
#define _POSE2D_H_

#include "isam/isam.h"
#include "isam/wall2d.h"

namespace MYSLAM {
    class Pose2d : public isam::Pose2d {
        public:

        Pose2d() : isam::Pose2d () {}
        Pose2d(double x, double y, double t) : isam::Pose2d (x, y, t) {}
        Pose2d(const Eigen::Vector3d& vec) : isam::Pose2d (vec) {}

        // sensor model for wall
        Wall2d transform_to(const Wall2d& p) const {
            double cost = cos(t());
            double sint = sin(t());

            double m = -p.u()/p.v();
            double c = (p.u()*p.u() + p.v()*p.v())/p.v();

            double m_ = (-sint + m*cost)/(cost + m*sint);
            double c_ = (c - y() + m*x())/(cost + m * sint);

            double u_ = -m_*c_/(m_*m_+1);
            double v_ = c_/(m_*m_+1);

            return Wall2d(u_,v_);
        }

        // inverse sensor model for wall
        Wall2d transform_from(const Wall2d& p) const {
            double cost = cos(t());
            double sint = sin(t());

            double m_ = -p.u()/p.v();
            double c_ = (p.u()*p.u() + p.v()*p.v())/p.v();

            double m = (sint +  m_*cost)/(cost - m_*sint);
            double c = -m*x() + y() + c_/(cost - m_ * sint);

            double u = -m*c/(m*m+1);
            double v = c/(m*m+1);

            return Wall2d(u,v);
        }
    };
}

#endif
