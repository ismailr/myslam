#ifndef _POSE2D_WALL2D_FACTOR_H_
#define _POSE2D_WALL2D_FACTOR_H_

#include <string>
#include <sstream>
#include <Eigen/Dense>

#include "isam/isam.h"
#include "isam/wall2d.h"
#include "isam/pose2d.h"

using namespace isam;

namespace MYSLAM {
    typedef NodeT<Wall2d> Wall2d_Node;
    typedef NodeT<Pose2d> Pose2d_Node;

    class Pose2d_Wall2d_Factor : public FactorT<Wall2d> {
        Pose2d_Node *_pose;
        Wall2d_Node *_wall;

        public:
        Pose2d_Wall2d_Factor (Pose2d_Node* pose, Wall2d_Node* wall, 
                const Wall2d& measure, const Noise& noise)
            : FactorT<Wall2d> ("Pose2d_Point2d_Factor", 2, noise, measure),
            _pose (pose), _wall (wall)
        {
            _nodes.resize(2);
            _nodes[0] = pose;
            _nodes[1] = wall;

        }

        void initialize() {
            require(_pose->initialized(), "Pose2d_Wall2d_Factor requires pose to be initialized");
            if (!_wall->initialized()) {
                Pose2d p = _pose->value();
                Wall2d predict = p.transform_from(_measure);
                _wall->init(predict);
            }
        }

        Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
            Pose2d po(_nodes[0]->vector(s));
            Wall2d wa(_nodes[1]->vector(s));
            Wall2d p = po.transform_to(wa);
            Eigen::VectorXd predicted = p.vector();
            return (predicted - _measure.vector());
        }

//        Jacobian jacobian() {
//            Pose2d po = _pose->value0();
//            Wall2d wa = _wall->value0();
//            double c = cos(po.t());
//            double s = sin(po.t());
//            double dx = wa.u() - po.x();
//            double dy = wa.v() - po.y();
//            // f(x)
//            double x =  c*dx + s*dy; // relative forward position of landmark point from pose
//            double y = -s*dx + c*dy; // relative position to the left
//
//            Eigen::MatrixXd M1(2,3);
//            M1 <<   -c, -s,  y,
//                    s,  -c, -x;
//            M1 = sqrtinf() * M1;
//
//            Eigen::MatrixXd M2(2,2);
//            M2 <<   c,   s,
//                    -s,  c;
//            M2 = sqrtinf() * M2;
//
//            Wall2d p(x, y);
//            Eigen::VectorXd r = sqrtinf() * (p.vector() - _measure.vector());
//            Jacobian jac(r);
//            jac.add_term(_pose, M1);
//            jac.add_term(_wall, M2);
//            return jac;
//        }
    };
}

#endif
