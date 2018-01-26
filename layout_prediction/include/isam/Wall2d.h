#ifndef _WALL2D_H_
#define _WALL2D_H_

#include <Eigen/Dense>

using namespace Eigen;

namespace Eigen {
    typedef Matrix<bool, Dynamic, 1> VectorXb;
}

namespace isam {
    class Wall2d {
        friend std::ostream& operator<<(std::ostream& out, const Wall2d& p) {
            p.write(out);
            return out;
        }

        double _u;
        double _v;

        public:
            static const int dim = 2;
            static const int size = 2;
            static const char* name() { return "Wall2d"; }

            Wall2d() : _u(0), _v(0) {}
            Wall2d(double u, double v) : _u(u), _v(v) {}
            Wall2d(const Eigen::Vector2d& vec) : _u(vec(0)), _v(vec(1)) {}

            double u() const {return _u;}
            double v() const {return _v;}

            void set_u(double u) {_u = u;}
            void set_v(double v) {_v = v;}

        Wall2d exmap(const Eigen::Vector2d& delta) const {
            Wall2d res = *this;
            res._u += delta(0);
            res._v += delta(1);
            return res;
        }

        Eigen::Vector2d vector() const {
            Eigen::Vector2d vec(_u, _v);
            return vec;
        }

        void set(double u, double v) {
            _u = u;
            _v = v;
        }

        void set(const Eigen::Vector2d& vec) {
            _u = vec(0);
            _v = vec(1);
        }

        void write(std::ostream &out) const {
            out << "(" << _u << ", " << _v << ")";
        }
      
        Eigen::VectorXb is_angle() const {
            return Eigen::VectorXb::Zero(size);
        }
    };
}

#endif
