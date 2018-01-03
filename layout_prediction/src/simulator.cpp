#include <random>
#include <iostream>
#include <fstream>
#include <limits>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/vertex_point_xy.h>  
#include <g2o/types/slam2d/vertex_se2.h>  
#include <g2o/types/slam2d/edge_se2.h>  
#include <g2o/types/slam2d/edge_se2_pointxy.h>  

#include "layout_prediction/simulator.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/wall_measurement.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/settings.h"

using namespace Eigen;
using namespace std;

template<typename T> T gaussian_generator (T mean, T dev)
{
    random_device rd;
    mt19937 mt(rd());
    normal_distribution<double> dist(mean, dev);
    return dist(mt);
}

template<typename T> T uniform_generator (T min, T max)
{
    random_device rd;
    mt19937 mt(rd());
    uniform_real_distribution<double> dist(min, max);
    return dist(mt);
}

namespace MYSLAM {
    void localToGlobal (Eigen::Vector2d& mc_, Pose::Ptr& pose, Eigen::Vector2d& mc)
    {
        double x = pose->_pose[0];
        double y = pose->_pose[1];
        double p = pose->_pose[2];
        double cosp = cos(p);
        double sinp = sin(p);

        double& m = mc[0];
        double& c = mc[1];
        double& m_ = mc_[0];
        double& c_ = mc_[1];

        m = (sinp +  m_ * cosp)/(cosp - m_ * sinp);
        c = -m * x + y + c_/(cosp - m_ * sinp);
    }

    void measurement_model (PoseVertex* p, WallVertex* w, double* z)
    {
        double x = p->estimate().translation().x();
        double y = p->estimate().translation().y();
        double t = p->estimate().rotation().angle();
        double sint = sin(t);
        double cost = cos(t);

        double xx = w->estimate().x();
        double xy = w->estimate().y();

        double m = -xx/xy;
        double c = (xx*xx + xy*xy)/xy;

        double m_ = (-sint + m*cost)/(cost + m*sint);
        double c_ = (c - y + m*x)/(cost + m*sint);

        double xx_ = -m_*c_/(m_*m_+1);
        double xy_ = c_/(m_*m_+1);

        z[0] = xx_;
        z[1] = xy_;
    }

    void measurement_model_xy (VertexSE2* p, VertexPointXY* w, double* z)
    {
        double px = p->estimate().translation().x();
        double py = p->estimate().translation().y();
        double pt = p->estimate().rotation().angle();
        double sint = sin(pt);
        double cost = cos(pt);

        double x = w->estimate().x();
        double y = w->estimate().y();

        Eigen::Vector2d result = p->estimate().inverse() * w->estimate();

        z[0] = result[0];
        z[1] = result[1];
    }
}

namespace MYSLAM {
    Simulator::Robot::Robot(Simulator* sim) : _sim (sim) 
    {
        truePose.setZero(); 
        simPose.setZero();
        truePose[2] = 2.68;
        simPose[2] = truePose[2];
        sense();
    }

    void Simulator::Robot::move()
    {
        sensedData.clear();
        sensedDinding.clear();

        const double tstep = 0.2;

        double xtrue = truePose[0];
        double ytrue = truePose[1];
        double ptrue = truePose[2];
        double dx = xtrue;
        double dy = ytrue;
        xtrue += tstep * cos (ptrue);
        ytrue += tstep * sin (ptrue);
        dx = xtrue - dx;
        dy = ytrue - dy;
        double d = sqrt ((dx*dx) + (dy*dy));

        truePose << xtrue, ytrue, ptrue;

        double noise = gaussian_generator<double>(0.0, xnoise_var);
        d = d + noise;

        double xsim = simPose[0];
        double ysim = simPose[1];
        double psim = simPose[2];
        psim = ptrue;
        xsim = xsim + d * cos(psim);
        ysim = ysim + d * sin(psim);

        simPose << xsim, ysim, psim;
    }

    void Simulator::Robot::sense()
    {
        double x = simPose[0];
        double y = simPose[1];
        double p = simPose[2];
        double cosp = cos(p);
        double sinp = sin(p);

        // sensor model
        for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
        {
            double& m = (*it).m;
            double& c = (*it).c;

            double m_ = (-sinp + m * cosp)/(cosp + m * sinp);
            double c_ = (c - y + m * x)/(cosp + m * sinp);

            double xxnoise = gaussian_generator<double>(0.0, wnoise_var);
            double xynoise = gaussian_generator<double>(0.0, wnoise_var);

            double xx_ = -m_*c_/(m_*m_+1) + xxnoise;
            double xy_ = c_/(m_*m_+1) + xynoise;

            Eigen::Vector2d data (xx_,xy_);

            double r = sqrt (xx_*xx_ + xy_*xy_);

            if (r <= RANGE)
            {
                sensedDinding.push_back (&(*it));
                sensedData.push_back (data);
            }
        }
    }

    Simulator::Simulator()
    {
        double grads[8] = {2, -.5, 2, -.5, 2, -.5, 2, -.5};
        double intercepts[8] = {30.0, 80.0, -30.0, 40.0, 15.0, 10.0, -30.0, -12.0};
        double x[9] = {-16.8,20.0,44.0,28.0,10.0,-2.0,16.0,7.2, -16.8};
        double y[9] = {-3.6,70.0,58.0,26.0,35.0,11.0,2.0,-15.6, -3.6};

        for (int i = 0; i < 1 /*NUM_OF_WALLS*/; i++)
        {
            Dinding *d = new Dinding;
            d->id = i;
            d->m = grads[i];
            d->c = intercepts[i];
            d->xx = -d->m*d->c/(d->m*d->m+1);
            d->xy = d->c/(d->m*d->m+1);
            Vector2d p (x[i], y[i]);
            Vector2d q (x[i+1], y[i+1]);
            d->p = p;
            d->q = q;
            struktur.push_back(*d);
        }

        robot = new Robot(this);
    }

    void Simulator::getNextState()
    {
        robot->move();
        robot->sense();
    }

    void Simulator::run()
    {
        MYSLAM::Graph graph;
        Optimizer o(graph);

        ofstream mfile;
        mfile.open ("/home/ism/tmp/sim.dat", ios::out | ios::app);

        std::vector<Dinding*> dindings;
        std::vector<Wall::Ptr> walls;

        for (int frame = 1; frame < MYSLAM::SIM_NUMBER_OF_ITERATIONS; frame++)
        {
            // poses
            Eigen::Vector3d t = robot->truePose;
            Eigen::Vector3d s = robot->simPose;

            mfile   << s[0] << " " << s[1] << " " << s[2] << " " << t[0] << " " << t[1] << " " << t[2] << std::endl;

            Pose::Ptr pose (new Pose);
            pose->_pose = s;
            pose->_poseByModel = t;
            graph._poseMap [pose->_id] = pose;
            graph._activePoses.push_back (pose->_id);

            for (size_t i = 0; i < robot->sensedDinding.size(); i++)
            {
                // asosiasi data
                Wall::Ptr w = NULL;
                Dinding* d = robot->sensedDinding[i];

                if (dindings.size() == 0)
                {
                    Wall::Ptr wall (new Wall);
                    w = wall;
                    dindings.push_back (d);
                    walls.push_back (w);
                } else {
                    for (int j = 0; j < dindings.size(); j++)
                    {
                        if (d->id == dindings[j]->id)
                        {
                            w = walls[j];
                            break;
                        }
                        else if (j == dindings.size() - 1)
                        {
                            Wall::Ptr wall (new Wall);
                            w = wall;
                            dindings.push_back (robot->sensedDinding[i]);
                            walls.push_back (w);
                            break;
                        }
                    }
                }

                double& xx_ = robot->sensedData[i][0];
                double& xy_ = robot->sensedData[i][1];
                double m_, c_;

                if (xy_ != 0)
                {
                    m_ = -xx_/xy_;
                    c_ = (xx_*xx_ + xy_*xy_)/xy_;
                } else {
                    m_ = std::numeric_limits<double>::infinity();
                    c_ = std::numeric_limits<double>::infinity();
                }

                Eigen::Vector2d mc_ (m_,c_);

                Eigen::Vector2d mc;
                localToGlobal (mc_, pose, mc);

                double& m = mc[0];
                double& c = mc[1];

                double xx = (-m*c)/(m*m+1);
                double xy = c/(m*m+1);

                w->_line.xx[0] = xx;
                w->_line.xx[1] = xy;

                graph._wallMap [w->_id] = w;
                graph._activeWalls.insert (w->_id);

                std::tuple<int, int> e (pose->_id, w->_id);
                graph._poseWallMap [e] = robot->sensedData[i];
                graph._activeEdges.push_back (e);
            }

            if (frame % 5 == 0)
            {
                o.localOptimize();
            }

            getNextState();
        }

        mfile.close();

        ofstream wfile;
        wfile.open ("/home/ism/tmp/wall.dat", std::ios::out | std::ios::app);
        wfile << "RESULT" << std::endl;
        for (std::map<int, Wall::Ptr>::iterator it = graph._wallMap.begin();
                it != graph._wallMap.end(); it++)
        {
            wfile << it->first << " " << it->second->_line.xx[0] << " " << it->second->_line.xx[1] << std::endl;
        }
        wfile << std::endl;

        for (int i = 0; i < struktur.size(); i++)
        {
            wfile << struktur[i].id << " " << struktur[i].xx << " " << struktur[i].xy << std::endl;
        }
        wfile << std::endl;
        wfile.close();

    }

    void Simulator::runsimple()
    {
        typedef BlockSolver< BlockSolverTraits<-1,-1> > SlamBlockSolver;
        typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

        g2o::SparseOptimizer *o = new g2o::SparseOptimizer;
        auto linearSolver = g2o::make_unique<SlamLinearSolver>();
        linearSolver->setBlockOrdering (false);
        OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg (
                g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
        o->setAlgorithm (solver);
        o->setVerbose (true);

        // noise
        double xnoise_var = 1.0;
        double ynoise_var = 1.0;
        double pnoise_var = 2.0 * M_PI/180.0;
        double wnoise_var = 1.0;

        // wall --> y = 2x + 6
        //      --> m = 2, c =6
        //      --> xx = -2.4, xy = 1.2
        double xxnoise = gaussian_generator<double>(0.0, wnoise_var);
        double xynoise = gaussian_generator<double>(0.0, wnoise_var);
        double wall[2] = {-2.4, 1.2};
        double xx = wall[0] + xxnoise;
        double xy = wall[1] + xynoise;
        double noisywall[2] = {xx,xy};

        WallVertex* w (new WallVertex);
        w->setId(3);
        w->setEstimateDataImpl (noisywall);
        o->addVertex(w);

        // 3 poses
        Eigen::Matrix<double, 3, 3> posepose;
        posepose << xnoise_var, 0, 0, 0, ynoise_var, 0, 0, 0, pnoise_var;

        SE2 p0 (-2.0,-2.0,M_PI/2);
        SE2 p1 (-2.0,1.0,M_PI/4);
        SE2 p2 (1.0,3.0,M_PI/4);
        SE2* p[3] = {&p0, &p1, &p2};

        SE2 np0, np1, np2;
        SE2* np[3] = {&np0, &np1, &np2};

        for (int i = 0; i < 3; i++)
        {
            double xnoise = gaussian_generator<double>(0.0, xnoise_var);
            double ynoise = gaussian_generator<double>(0.0, ynoise_var);
            double pnoise = gaussian_generator<double>(0.0, pnoise_var);
            double x = p[i]->translation().x() + xnoise;
            double y = p[i]->translation().y() + ynoise;
            double th = p[i]->rotation().angle() + pnoise;
            Eigen::Vector2d t (x,y);
            Eigen::Rotation2Dd r (th);
            np[i]->setTranslation (t);
            np[i]->setRotation (r);
        }

        PoseVertex* pv0 (new PoseVertex);
        PoseVertex* pv1 (new PoseVertex);
        PoseVertex* pv2 (new PoseVertex);
        pv0->setId(0); pv0->setEstimate (p0); pv0->setModel (p0); //pv0->setFixed (true);
        pv1->setId(1); pv1->setEstimate (np1); pv1->setModel (p1); 
        pv2->setId(2); pv2->setEstimate (np2); pv2->setModel (p2);
        o->addVertex (pv0);
        o->addVertex (pv1);
        o->addVertex (pv2);

        PoseMeasurement* pm0 = new PoseMeasurement;
        pm0->vertices()[0] = pv0;
        pm0->vertices()[1] = pv1;
        pm0->setMeasurement (pv0->estimate().inverse() * pv1->estimate());
        pm0->information () = posepose.inverse();
        o->addEdge (pm0);

        PoseMeasurement* pm1 = new PoseMeasurement;
        pm1->vertices()[0] = pv1;
        pm1->vertices()[1] = pv2;
        pm1->setMeasurement (pv1->estimate().inverse() * pv2->estimate());
        pm1->information () = posepose.inverse();
        o->addEdge (pm1);

        // wall measurement
        Eigen::Matrix<double, 2, 2> posewall;
        posewall << wnoise_var, 0, 0, wnoise_var;

        WallMeasurement* wm0 = new WallMeasurement;
        wm0->vertices()[0] = pv0;
        wm0->vertices()[1] = w;
        double z0[2]; measurement_model (pv0, w, z0);
        z0[0] += gaussian_generator<double>(0.0, wnoise_var);
        z0[1] += gaussian_generator<double>(0.0, wnoise_var);
        wm0->setMeasurementData (z0);
        wm0->information() = posewall.inverse();
        o->addEdge (wm0);

        WallMeasurement* wm1 = new WallMeasurement;
        wm1->vertices()[0] = pv1;
        wm1->vertices()[1] = w;
        double z1[2]; measurement_model (pv1, w, z1);
        z1[0] += gaussian_generator<double>(0.0, wnoise_var);
        z1[1] += gaussian_generator<double>(0.0, wnoise_var);
        wm1->setMeasurementData (z1);
        wm1->information() = posewall.inverse();
        o->addEdge (wm1);

        WallMeasurement* wm2 = new WallMeasurement;
        wm2->vertices()[0] = pv2;
        wm2->vertices()[1] = w;
        double z2[2]; measurement_model (pv2, w, z2);
        z2[0] += gaussian_generator<double>(0.0, wnoise_var);
        z2[1] += gaussian_generator<double>(0.0, wnoise_var);
        wm2->setMeasurementData (z2);
        wm2->information() = posewall.inverse();
        o->addEdge (wm2);

        o->save("/home/ism/tmp/simplesim_before.g2o");
        o->initializeOptimization();
        o->optimize(10);
        o->save("/home/ism/tmp/simplesim_after.g2o");
    }

    void Simulator::runsimple2()
    {
        typedef BlockSolver< BlockSolverTraits<-1,-1> > SlamBlockSolver;
        typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

        g2o::SparseOptimizer *o = new g2o::SparseOptimizer;
        auto linearSolver = g2o::make_unique<SlamLinearSolver>();
        linearSolver->setBlockOrdering (false);
        OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg (
                g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
        o->setAlgorithm (solver);
        o->setVerbose (true);

        // noise
        double xnoise_var = 1.0;
        double ynoise_var = 1.0;
        double pnoise_var = 2.0 * M_PI/180.0;

        double point[2] = {0.0, 0.0};
        point[0] += gaussian_generator<double>(0.0, xnoise_var);
        point[1] += gaussian_generator<double>(0.0, ynoise_var);

        VertexPointXY* w (new VertexPointXY);
        w->setId(3);
        w->setEstimateDataImpl (point);
        o->addVertex(w);

        // 3 poses
        Eigen::Matrix<double, 3, 3> posepose;
//        posepose << xnoise_var, 0, 0, 0, ynoise_var, 0, 0, 0, pnoise_var;
        posepose.setIdentity();

        SE2 p0 (-2.0,-2.0,M_PI/2);
        SE2 p1 (-2.0,1.0,M_PI/4);
        SE2 p2 (1.0,3.0,M_PI/4);
        SE2* p[3] = {&p0, &p1, &p2};

        SE2 np0, np1, np2;
        SE2* np[3] = {&np0, &np1, &np2};

        for (int i = 0; i < 3; i++)
        {
            double xnoise = gaussian_generator<double>(0.0, xnoise_var);
            double ynoise = gaussian_generator<double>(0.0, ynoise_var);
            double pnoise = gaussian_generator<double>(0.0, pnoise_var);
            double x = p[i]->translation().x() + xnoise;
            double y = p[i]->translation().y() + ynoise;
            double th = p[i]->rotation().angle() + pnoise;
            Eigen::Vector2d t (x,y);
            Eigen::Rotation2Dd r (th);
            np[i]->setTranslation (t);
            np[i]->setRotation (r);
        }

        VertexSE2* pv0 (new VertexSE2);
        VertexSE2* pv1 (new VertexSE2);
        VertexSE2* pv2 (new VertexSE2);
        pv0->setId(0); pv0->setEstimate (p0); 
        pv1->setId(1); pv1->setEstimate (np1); 
        pv2->setId(2); pv2->setEstimate (np2);
        o->addVertex (pv0);
        o->addVertex (pv1);
        o->addVertex (pv2);

        EdgeSE2* pm0 = new EdgeSE2;
        pm0->vertices()[0] = pv0;
        pm0->vertices()[1] = pv1;
        pm0->setMeasurement (pv0->estimate().inverse() * pv1->estimate());
        pm0->information () = posepose.inverse();
        o->addEdge (pm0);

        EdgeSE2* pm1 = new EdgeSE2;
        pm1->vertices()[0] = pv1;
        pm1->vertices()[1] = pv2;
        pm1->setMeasurement (pv1->estimate().inverse() * pv2->estimate());
        pm1->information () = posepose.inverse();
        o->addEdge (pm1);

        // point measurement
        Eigen::Matrix<double, 2, 2> posepoint;
//        posepoint << xnoise_var, 0, 0, ynoise_var;
        posepoint.setIdentity();

        EdgeSE2PointXY* wm0 = new EdgeSE2PointXY;
        wm0->vertices()[0] = pv0;
        wm0->vertices()[1] = w;
        double z0[2]; measurement_model_xy (pv0, w, z0);
        z0[0] += gaussian_generator<double>(0.0, xnoise_var);
        z0[1] += gaussian_generator<double>(0.0, ynoise_var);
        wm0->setMeasurementData (z0);
        wm0->information() = posepoint.inverse();
        o->addEdge (wm0);

        EdgeSE2PointXY* wm1 = new EdgeSE2PointXY;
        wm1->vertices()[0] = pv1;
        wm1->vertices()[1] = w;
        double z1[2]; measurement_model_xy (pv1, w, z1);
        z1[0] += gaussian_generator<double>(0.0, xnoise_var);
        z1[1] += gaussian_generator<double>(0.0, ynoise_var);
        wm1->setMeasurementData (z1);
        wm1->information() = posepoint.inverse();
        o->addEdge (wm1);

        EdgeSE2PointXY* wm2 = new EdgeSE2PointXY;
        wm2->vertices()[0] = pv2;
        wm2->vertices()[1] = w;
        double z2[2]; measurement_model_xy (pv2, w, z2);
        z2[0] += gaussian_generator<double>(0.0, xnoise_var);
        z2[1] += gaussian_generator<double>(0.0, ynoise_var);
        wm2->setMeasurementData (z2);
        wm2->information() = posepoint.inverse();
        o->addEdge (wm2);

        o->save("/home/ism/tmp/simplesim2_before.g2o");
        o->initializeOptimization();
        o->optimize(10);
        o->save("/home/ism/tmp/simplesim2_after.g2o");
    }
}
