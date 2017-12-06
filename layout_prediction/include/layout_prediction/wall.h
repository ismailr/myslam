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

#ifndef _WALL_H_
#define _WALL_H_

#include <string>

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/stuff/misc.h"
#include "g2o/types/slam2d/vertex_point_xy.h"

#include "vertex_line2d.h"

using namespace g2o;

class Wall : public BaseVertex <2, Line2D>
{
    public:
    typedef std::shared_ptr<Wall> Ptr;
    typedef std::shared_ptr<const Wall> ConstPtr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Wall (); // Set line params to zero
    Wall (double rho, double theta); // Set line params
    Wall (double rho, double theta, Eigen::Vector2d p, Eigen::Vector2d q); // Set line params and two edges

    double theta() const {return _estimate[0]; }
    void setTheta(double t) { _estimate[0] = t; }
    double thetaGlobal() const { return _thetaGlobal; }
    void setThetaGlobal (double t) { _thetaGlobal = t; }

    double rho() const {return _estimate[1]; }
    void setRho(double r) { _estimate[1] = r; }
    double rhoGlobal () const {return _rhoGlobal; }
    void setRhoGlobal (double r) { _rhoGlobal = r; }

    Eigen::Vector2d p() const {return _p;}
    Eigen::Vector2d q() const {return _q;}
    Eigen::Vector2d center() const {return _pq;};
    int getPose() const { return _observerPoses.front(); }
    std::vector<int> getObserverPoses () const { return _observerPoses; } 

    virtual void setToOriginImpl() {
        _estimate.setZero();
    }

    virtual bool setEstimateDataImpl(const double* est){
        Eigen::Map<const Vector2D> v(est);
        _estimate=Line2D(v);
        return true;
    }

    virtual bool getEstimateData(double* est) const{
        Eigen::Map<Vector2D> v(est);
        v=_estimate;
        return true;
    }

    virtual int estimateDimension() const {
        return 2;
    }

    virtual bool setMinimalEstimateDataImpl(const double* est){
        return setEstimateData(est);
    }

    virtual bool getMinimalEstimateData(double* est) const{
        return getEstimateData(est);
    }

    virtual int minimalEstimateDimension() const {
        return 2;
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate += Eigen::Map<const Vector2D>(update);
        _estimate(0) = normalize_theta(_estimate(0));
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    int p1Id, p2Id;

    double getFitness ();
    void setFitness (double fitness);

    void setObserverPose (int);

    private:
    Eigen::Vector2d _p,_q /* edges */, _pq /* center of p and q */; 
    double _fitness; // fitness of wall as a result of line-fitting process
    std::vector <int> _observerPoses;
    double _rhoGlobal, _thetaGlobal;
};

class Wall2 : public VertexLine2D
{
    // Constructor
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        std::string nodetype; 
        int simId;

        typedef std::shared_ptr<Wall2> Ptr;
        typedef std::shared_ptr<const Wall2> ConstPtr;
        Wall2();
//        Wall2(double rho, double theta);
        Wall2(double gradient, double intercept);

        void updateData ();

    // Gradient, intercept
    private:
        double _gradient;
        double _intercept;

    // EndPoints and Inliers
    public:
        typedef std::tuple<Eigen::Vector2d, Eigen::Vector2d> EndPoints;
        typedef std::vector<Eigen::Vector3d> Inliers;

        void set_gradient_intercept(double g, double i) { _gradient=g; _intercept=i; };
        void set_inliers (Inliers inliers); 
        EndPoints getEndPoints() { return _endPoints; };
        Eigen::Vector2d get_center_point () { return _centerPoint; };

        virtual void oplusImpl(const double* update)
        {
//            std::cout << "UPDATE: " << update[0] << " " << update[1] << std::endl;
            _estimate += Eigen::Map<const Vector2D>(update);
            _estimate(0) = normalize_theta(_estimate(0));
        }

    private:
        EndPoints _endPoints;
        Eigen::Vector2d _centerPoint;
        Inliers _inliers;
        double _length;

        void calculate_edge_points ();
        void calculate_center_point ();
        void setLength (double l) { _length = l; };
        double getLength () const { return _length; };

    // Fitness
    public:
        double get_fitness () { return _fitness; };
    private:
        double _fitness;
        void calculate_fitness ();

    // Measurement
    public:
        void setMeasurement (Eigen::Vector2d measurement) { _measurement = measurement; };
        Eigen::Vector2d getMeasurement () const { return _measurement; };
    private:
        Eigen::Vector2d _measurement;
};

class Wall3 : public VertexPointXY
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef std::shared_ptr<Wall3> Ptr;
        typedef std::shared_ptr<const Wall3> ConstPtr;

        Wall3();

        void setm (double m) { _m = m; };
        void setc (double c) { _c = c; };
        double getm () const { return _m; };
        double getc () const { return _c; };
        void setpq (Eigen::Vector2d p, Eigen::Vector2d q) {_p = p; _q = q; };
        double getp () const { return _p; };
        double getq () const { return _q; };

    private:
        double _m, _c;
        Eigen::Vector2d _p, _q;

};

class Wall4 : public BaseVertex <2, Line2D>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Wall4> Ptr;
    typedef std::shared_ptr<const Wall4> ConstPtr;

    Wall4 (); // Set line params to zero
    Wall4 (double rho, double theta); // Set line params

    double theta() const {return _estimate[0]; }
    double rho() const {return _estimate[1]; }
    void setTheta(double t) { _estimate[0] = t; }
    void setRho(double r) { _estimate[1] = r; }

    virtual void setToOriginImpl() {
        _estimate.setZero();
    }

    virtual bool setEstimateDataImpl(const double* est){
        Eigen::Map<const Vector2D> v(est);
        _estimate=Line2D(v);
        return true;
    }

    virtual bool getEstimateData(double* est) const{
        Eigen::Map<Vector2D> v(est);
        v=_estimate;
        return true;
    }

    virtual int estimateDimension() const {
        return 2;
    }

    virtual bool setMinimalEstimateDataImpl(const double* est){
        return setEstimateData(est);
    }

    virtual bool getMinimalEstimateData(double* est) const{
        return getEstimateData(est);
    }

    virtual int minimalEstimateDimension() const {
        return 2;
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate += Eigen::Map<const Vector2D>(update);
        _estimate(0) = normalize_theta(_estimate(0));
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void setGrad (double m, double c) { _m = m; _c = c; }; 
    Eigen::Vector2d getGrad () const { return Eigen::Vector2d (_m,_c); };

    private:
    double _m, _c;
};

#endif
