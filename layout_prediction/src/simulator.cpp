#include <random>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>

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
#include "layout_prediction/point.h"
#include "layout_prediction/wall_measurement.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/settings.h"
#include "layout_prediction/helpers.h"

using namespace Eigen;
using namespace std;

namespace MYSLAM {

    Simulator::Robot::Robot(Simulator* sim) 
        : _sim (sim), _moveStep (0.5), nearestLandmark (100.0)
    {
        truePose.setZero(); 
        simPose.setZero();
        truePose[2] = M_PI;
        simPose[2] = truePose[2];
//        sense();
    }

    void Simulator::Robot::move() {

        if (nearestLandmark < 1.0) {
            double _rot = uniform_generator<double>(-M_PI, M_PI);
            int _dir100 = uniform_generator<int>(1,100);

            int _dir;
            _dir100 > 50 ? _dir = 1 : _dir = 0;

            turn (_dir, _rot);
            return;
        }

        SE2 pose;
        pose.fromVector (truePose);  

        SE2 move (_moveStep, 0.0, 0.0);
        pose = pose * move;
        truePose = pose.toVector();
//        truePath.push_back(truePose);

        SE2 simpose;
        simpose.fromVector(simPose);

        SE2 simmove (
                move.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
//                move.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
//                move.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
                0.0, 0.0);
        simpose = simpose * simmove;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::turn (int direction, double angle)
    {
        direction == 0 ? angle = angle : angle = -angle;
//        SE2 turn (_moveStep, 0.0, angle);
        SE2 turn (0.0, 0.0, angle);

        SE2 pose;
        pose.fromVector (truePose);
        pose = pose * turn;
        truePose = pose.toVector();

        SE2 simpose;
        simpose.fromVector (simPose);

        SE2 simturn (
                turn.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
//                turn.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
                0.0, 
//                0.0, 
                turn.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
        simpose = simpose * simturn;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::sense()
    {
        double _nearestLandmark = 100.0;

        sensedData.clear();
        sensedDinding.clear();
        sensedDataBenda.clear();
        sensedBenda.clear();

        SE2 simpose; simpose.fromVector (simPose);
        SE2 truepose; truepose.fromVector (truePose);

        // sensor model
        for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
        {
            double unoise = gaussian_generator<double>(0.0, wnoise_stdev);
            double vnoise = gaussian_generator<double>(0.0, wnoise_stdev);

            double u = (*it).xx;
            double v = (*it).xy;

            Eigen::Vector2d xx (u, v);
            Eigen::Vector2d xx_ = truepose.inverse() * xx;

            Eigen::Vector2d data (xx_[0] + unoise, xx_[1] + vnoise);
//
//            double r = sqrt (xx_[0]*xx_[0] + xx_[1]*xx_[1]);
//            double angle = normalize_theta (atan2 (xx_[1],xx_[0]));
//
            double m = (*it).m;
            double c = (*it).c;

            double& x = truePose[0];
            double& y = truePose[1];
            double& t = truePose[2];
            double cost = cos(t);
            double sint = sin(t);

            double m_ = (-sint+m*cost)/(cost+m*sint);
            double c_ = (c-y+m*x)/(cost+m*sint);

            double u_ = (-m_*c_)/(m_*m_+1) + unoise;
            double v_ = c_/(m_*m_+1) + vnoise;

            double r = sqrt (u_*u_ + v_*v_);
//            double r_ = std::abs (-c_/m_);            
            double angle = normalize_angle (atan2 (v_,u_));

            if (r <= RANGE)
            {
               if ((angle >= 0.0 && angle < 0.5*M_PI) || (angle > 1.5*M_PI && angle <= 2*M_PI))
                {
                    if (r< _nearestLandmark) _nearestLandmark = r;
                    sensedDinding.push_back (&(*it));
                    sensedData.push_back (data);
                }
            }
        }

        for (std::vector<Benda>::iterator it = _sim->bendabenda.begin(); it != _sim->bendabenda.end(); it++)
        {
            double oxnoise = gaussian_generator<double>(0.0, 1.0e-2);
            double oynoise = gaussian_generator<double>(0.0, 1.0e-2);
            double otnoise = gaussian_generator<double>(0.0, 2.0*M_PI/180.0);

            SE2 objectPose; objectPose.fromVector (it->pose);

            SE2 objectMeasurement = truepose.inverse() * objectPose;

            SE2 objectMeasurementWithNoise (
                   objectMeasurement.toVector()[0] + oxnoise, 
                   objectMeasurement.toVector()[1] + oynoise, 
                   objectMeasurement.toVector()[2] + otnoise); 

            double& omx = objectMeasurementWithNoise.toVector()[0];
            double& omy = objectMeasurementWithNoise.toVector()[1];
            double& omt = objectMeasurementWithNoise.toVector()[2];

            double r = sqrt (omx*omx + omy*omy);
            double angle = normalize_theta (atan2 (omy,omx));

//            if (r < _nearestLandmark) _nearestLandmark = r;

            if (r <= RANGE)
            {
                if ((angle >= 0.0 && angle < M_PI/2) || (angle > -2*M_PI && angle <= -M_PI/2))
                {
                    sensedBenda.push_back (&(*it));
                    sensedDataBenda.push_back (objectMeasurementWithNoise.toVector());
                }
            }
        }

        nearestLandmark = _nearestLandmark;
    }

    void Simulator::Robot::observe()
    {
        // observation noise
        Eigen::Matrix2d R;
        double var = wnoise_stdev * wnoise_stdev;
        R << var, 0, 0, var;
                                
        double sigma_weight = 0.0; // sum of weights
        for (int i = 0; i < _sim->N; i++)
        {
            // current pose
            SE2 pose = _sim->particles[i].path.back(); 

            // observations
            std::vector<std::tuple<int, Eigen::Vector2d> > observations;
            for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
            {
                double unoise = gaussian_generator<double>(0.0, wnoise_stdev);
                double vnoise = gaussian_generator<double>(0.0, wnoise_stdev);
                double pnoise = gaussian_generator<double>(0.0, pnoise_stdev);

                Eigen::Vector2d globalWall ((*it).xx,(*it).xy);
//                Eigen::Vector2d localWall = pose.inverse() * globalWall;

                Eigen::Vector2d localWall; 

                // sensor model for range-bearing
                double xx = (*it).xx;
                double xy = (*it).xy;
                double x = pose.translation().x();
                double y = pose.translation().y();
                double t = pose.rotation().angle();
                localWall[0] = sqrt ((xx - x)*(xx-x) + (xy-y)*(xy-y)) + unoise; 
                localWall[1] = atan2 (xy-y,xx-x) - t + pnoise;
                
//                localWall[0] += unoise;
//                localWall[1] += vnoise;

//                double r = sqrt (localWall[0]*localWall[0] + localWall[1]*localWall[1]);
//                double angle = normalize_angle (atan2 (localWall[1],localWall[0]));

                double& r = localWall[0];
                if (r <= RANGE)
                {
//                    if ((angle >= 0.0 && angle < M_PI/2) || (angle > 3*M_PI/2 && angle <= 2*M_PI)) //landmark observed
//                    {
                        std::tuple<int, Eigen::Vector2d> obs 
                            = std::make_tuple ((*it).id, localWall);
                        observations.push_back (obs);
//                    }
                }
            }

            // data association and weight update
            double weight;
            for (std::vector<std::tuple<int, Eigen::Vector2d> >::iterator it = observations.begin();
                    it != observations.end(); it++)
            {
                int id = std::get<0>(*it);
                Eigen::Vector2d z = std::get<1>(*it);

                auto jt = find_if ( _sim->particles[i].landmarks.begin(),
                                    _sim->particles[i].landmarks.end(),
                                    [id](const std::tuple<int, Dinding, Eigen::Matrix2d>& e)
                                    { return std::get<0>(e) == id; });

                if (jt == _sim->particles[i].landmarks.end()) {  // new landmark
                    // inverse sensor model
//                    Eigen::Vector2d observedWall = pose * z;
                    Eigen::Vector2d observedWall;
                    observedWall[0] = z[0] * cos (z[1]);
                    observedWall[1] = z[0] * sin (z[1]);

                    Dinding d;
                    d.id = id;
                    d.xx = observedWall[0];
                    d.xy = observedWall[1];
                    Eigen::Matrix2d jac = *obvJacobian (pose, z);
                    jac = jac.transpose() * R.inverse() * jac;
                    std::tuple<int, Dinding, Eigen::Matrix2d> l 
                        = std::make_tuple (d.id, d, jac.inverse());
                    _sim->particles[i].landmarks.push_back (l);
                } else {                                        // update landmark
                    Eigen::Vector2d landmark;
                    landmark << std::get<1>(*jt).xx, std::get<1>(*jt).xy;
                    Eigen::Vector2d z_hat = pose.inverse() * landmark;
                    Eigen::Vector2d inov = (z - z_hat);

                    Eigen::Matrix2d S = std::get<2>(*jt);
                    Eigen::Matrix2d G = *obvJacobian (pose, landmark);
                    Eigen::Matrix2d Z = G * S * G.transpose() + R;
                    Eigen::Matrix2d K = S * G * Z.inverse();

                    Eigen::Vector2d updatedLandmark = landmark + K * inov;
                    Eigen::Matrix2d updatedCov = (Eigen::Matrix2d::Identity() - K * G) * S;

                    std::get<1>(*jt).xx = updatedLandmark[0];
                    std::get<1>(*jt).xy = updatedLandmark[1];
                    std::get<2>(*jt) = updatedCov;

                    double norm = 1/sqrt(std::abs(2*M_PI*Z.determinant()));
                    norm == 0 ? norm = 0.0 : norm = log10(norm);

                    double ex = -0.5 * inov.transpose() * Z.inverse() * inov;
                    double w = norm + ex;
                    weight += w;
                }
            }

            _sim->particles[i].weight = weight;
            sigma_weight += weight;
        }

        // normalize weights
        for (int i = 0; i < _sim->N; i++)
            _sim->particles[i].weight /= sigma_weight;
    }

    Simulator::Simulator()
    {
        double rot = uniform_generator<double>(-M_PI, M_PI);
        Eigen::Vector3d r (0.0, 0.0, rot);
        M.fromVector(r);

        roomwidth = uniform_generator<double>(0.0, 10.0) + 5.0;
        roomlength = uniform_generator<double>(0.0, 10.0 ) + 5.0;

        const int N = 4;
        double grads[4] = {2, -.5, 2, -.5};
        double intercepts[4] = {25.0, 8.0, -10.0, -5.0};
        double x[5] = {-12, -6.8, 7.2, 2, -12};
        double y[5] = {1, 11.4, 4.4, -6, 1};

//        std::vector<double> grads (N); 
//        std::vector<double> intercepts (N); 
//        std::vector<double> xx (N); 
//        std::vector<double> xy (N); 
//        std::vector<double> x(N + 1); 
//        std::vector<double> y(N + 1); 
//
//        for (int i = 0; i < N; i++)
//        {
//            grads[i] = uniform_generator<double>(-5.0,5.0);
//            intercepts[i] = uniform_generator<double>(-10.0, 10.0);
//            xx[i] = uniform_generator<double>(-10.0,0.0);
//            xy[i] = uniform_generator<double>(0.0, 9.0);
//            x[i] = uniform_generator<double>(-5.0,5.0);
//            y[i] = uniform_generator<double>(-5.0,5.0);
//        }
//
//        x.push_back(x[0]);
//        y.push_back(y[0]);

//        const int N = MYSLAM::SIM_NUMBER_OF_LANDMARKS;

//        ofstream wallfile;
//        wallfile.open ("/home/ism/tmp/truewall2.dat", std::ios::out | std::ios::app);
//        for (int i = 0; i < N * 2; i = i + 2)
/*        for (int i = 0; i < N; i++)
        {
            Dinding *d = new Dinding;
            d->id = i;
            d->m = grads[i];
            d->c = intercepts[i];
            d->xx = -d->m*d->c/(d->m*d->m+1);
            d->xy = d->c/(d->m*d->m+1);
//            d->xx = xx[i];
//            d->xy = xy[i];

            Vector2d p (x[i], y[i]);
            Vector2d q (x[i+1], y[i+1]);
            d->p = p;
            d->q = q;
            struktur.push_back(*d);
//            wallfile << d->xx << " " << d->xy << std::endl;
        }*/
//        wallfile.close();

//        int M = 10;
//        ofstream objectfile;
//        objectfile.open ("/home/ism/tmp/trueobject.dat", std::ios::out | std::ios::app);
//        for (int i = 0, j = 0; i < M; i++, j = j + 3) {
//            Benda b;
//            b.id = i;
//            Eigen::Vector3d o (objects[j], objects[j+1], objects[j+2]);
//            b.pose = o;
//            bendabenda.push_back (b);
//            objectfile << o.transpose() << std::endl;
//        }
//        objectfile.close();

        generateRoom();
        generateObjects (MYSLAM::SIM_NUMBER_OF_OBJECTS);

        robot = new Robot(this);

    }

    void Simulator::getNextState()
    {
        robot->move();
        robot->sense();
    }

    void Simulator::getNextStatePF()
    {
        robot->sampleMove();
    }


    void Simulator::run()
    {
        MYSLAM::Graph graph;
        Optimizer o(graph);

        ofstream mfile;
        mfile.open ("/home/ism/tmp/sim.dat", ios::out | ios::app);

        std::vector<Dinding*> dindings;
        std::vector<Wall::Ptr> walls;
        std::vector<Benda*> bendas;
        std::vector<Object::Ptr> objects;

        Eigen::Vector3d lastSimPose;
        Pose::Ptr lastPose;

        double turn_angle[9] =  {90, 45, 75, 100, 90, 90,  90,  45, 135};
        int turn_time[9] =      {17, 28, 40,  66, 83, 93,  98, 103, 112};
        int turn_direction[9] = {1 , 1 ,  1,  1,  1,  1,   0,   0,   1};
        int turn_now = 0;

        // first pose
        mfile   << robot->simPose[0] << " " << robot->simPose[1] << " " << robot->simPose[2] << " " 
                << robot->truePose[0] << " " << robot->truePose[1] << " " << robot->truePose[2] << std::endl;
        Pose::Ptr firstPose (new Pose);
        firstPose->_pose = robot->simPose;
        graph._poseMap [firstPose->_id] = firstPose;
        graph._activePoses.push_back (firstPose->_id);
        lastSimPose = robot->simPose;
        lastPose = firstPose;

        // stats
        robot->truePath.push_back (robot->truePose);
        robot->finalpose.push_back (firstPose->_id); // end of stats

        getNextState();
//        robot->move(0);
//        robot->sense();

        for (int frame = 1; frame < MYSLAM::SIM_NUMBER_OF_ITERATIONS; frame++)
//        for (int frame = 1; frame < 200; frame++)
        {
            // poses
            Eigen::Vector3d t = robot->truePose;
            Eigen::Vector3d s = robot->simPose;

            mfile   << s[0] << " " << s[1] << " " << s[2] << " " 
                    << t[0] << " " << t[1] << " " << t[2] << std::endl;

            SE2 lastodom;
            lastodom.fromVector (lastSimPose);
            SE2 currentodom;
            currentodom.fromVector (s);

            SE2 delta = lastodom.inverse() * currentodom;

            SE2 lastpose;
            lastpose.fromVector (lastPose->_pose);
            Pose::Ptr pose (new Pose);
            pose->_pose = (lastpose * delta).toVector();

            graph._poseMap [pose->_id] = pose;
            graph._activePoses.push_back (pose->_id);

            // stats
            robot->truePath.push_back (t);
            robot->finalpose.push_back (pose->_id); // end of stats

            for (size_t i = 0; i < robot->sensedDinding.size(); i++)
            {
                // asosiasi data
                Wall::Ptr w = NULL;
                Dinding* d = robot->sensedDinding[i];
                bool newLandmark = true;

                if (dindings.size() == 0)
                {
                    Wall::Ptr wall (new Wall);
                    w = wall;
                    w->_line.p = d->p;
                    w->_line.q = d->q;
                    w->initParams();
                    dindings.push_back (d);
                    walls.push_back (w);

                } else {
                    for (int j = 0; j < dindings.size(); j++)
                    {
                        if (d->id == dindings[j]->id)
                        {
                            w = walls[j];
                            newLandmark = false;
                            break;
                        }
                        else if (j == dindings.size() - 1)
                        {
                            Wall::Ptr wall (new Wall);
                            w = wall;
                            w->_line.p = d->p;
                            w->_line.q = d->q;
                            w->initParams();
                            dindings.push_back (d);
                            walls.push_back (w);
                            break;
                        }
                    }
                }

                if (newLandmark) {
                    double& xx_ = robot->sensedData[i][0];
                    double& xy_ = robot->sensedData[i][1];

                    double x = pose->_pose[0];
                    double y = pose->_pose[1];
                    double t = pose->_pose[2];
                    double cost = cos(t);
                    double sint = sin(t);

                    w->_line.xx[0] = xx_*cost - xy_*sint + x;
                    w->_line.xx[1] = xx_*sint + xy_*cost + y;
                    w->initParams();
                }

                graph._wallMap [w->_id] = w;
                graph._activeWalls.insert (w->_id);

                std::tuple<int, int> e (pose->_id, w->_id);
                graph._poseWallMap [e] = robot->sensedData[i];
                graph._activeEdges.push_back (e);
            }

            for (size_t i = 0; i < robot->sensedBenda.size(); i++)
            {
                // asosiasi data
                Object::Ptr o = NULL;
                Benda* b = robot->sensedBenda[i];
                bool newObject = true;

                if (bendas.size() == 0)
                {
                    Object::Ptr object (new Object);
                    o = object;
                    bendas.push_back (b);
                    objects.push_back (o);

                } else {
                    for (int j = 0; j < bendas.size(); j++)
                    {
                        if (b->id == bendas[j]->id)
                        {
                            o = objects[j];
                            newObject = false;
                            break;
                        }
                        else if (j == bendas.size() - 1)
                        {
                            Object::Ptr object (new Object);
                            o = object;
                            bendas.push_back (b);
                            objects.push_back (o);
                            break;
                        }
                    }
                }

                if (newObject) {
                    Eigen::Vector3d m = robot->sensedDataBenda[i];
                    SE2 measurement; measurement.fromVector (m); 
                    SE2 robotPose; robotPose.fromVector (pose->_pose);
                    SE2 objectPose = robotPose * measurement;
                    o->_pose = objectPose.toVector();
                }

                graph._objectMap [o->_id] = o;
                graph._activeObjects.insert (o->_id);

                std::tuple<int, int> e (pose->_id, o->_id);
                graph._poseObjectMap [e] = robot->sensedDataBenda[i];
                graph._activePoseObjectEdges.push_back (e);
            }

            int N = graph._activePoses.size();
            int M = graph._activeWalls.size();
            int O = graph._activeObjects.size();
            int P = graph._activePoseObjectEdges.size();
            int E = graph._activeEdges.size() + N - 1 + P;
            int V = 3*N + 2*M + 3*O;
//            std::ofstream lfile;
//            lfile.open ("/home/ism/tmp/numoflandmark.dat",std::ios::out | std::ios::app);
//            lfile   << frame << " " 
//                    << N << " " << M << " " << E << " " << V << std::endl;
//            lfile.close();

//            if (frame % 5 == 0)
//            if (graph._activeWalls.size() >= 2 && graph._activePoses.size() >= 2)
//            if (E >= V)
            if ((graph._activeWalls.size() >= 1 && graph._activePoses.size() >= 9 && graph._activeObjects.size() >= 2)
              || frame + 1 == MYSLAM::SIM_NUMBER_OF_ITERATIONS)
            {
                o.localOptimize();
            }

            lastSimPose = s;
            lastPose = pose;

            robot->move();
            robot->sense();

//            if (frame == turn_time[turn_now]) {
//                robot->turn (turn_direction[turn_now], turn_angle[turn_now] * M_PI/180.0);
//                robot->sense();
//                turn_now++;
//            } else {
//                robot->move();
//                robot->sense();
//            }
        }

//        o.globalOptimize();

        mfile.close();

        ofstream wfile;
        wfile.open ("/home/ism/tmp/wall.dat", std::ios::out | std::ios::app);
        for (std::map<int, Wall::Ptr>::iterator it = graph._wallMap.begin();
                it != graph._wallMap.end(); it++)
        {
            wfile << it->second->_line.p.transpose() << " " << it->second->_line.q.transpose() << std::endl;
        }
        wfile.close();

        ofstream ofile;
        ofile.open ("/home/ism/tmp/object.dat", std::ios::out | std::ios::app);
        for (std::map<int, Object::Ptr>::iterator it = graph._objectMap.begin();
                it != graph._objectMap.end(); it++)
        {
            ofile << it->second->_pose.transpose() << std::endl;
        }
        ofile.close();

        calculateRMSE (graph);
    }

    void Simulator::Robot::sampleMove (int direction, double angle)
    {
        SE2 pose; pose.fromVector(truePose);
        SE2 simpose; simpose.fromVector(simPose);

        SE2 move;
        switch (direction) {
            case 0: move.setTranslation (Eigen::Vector2d (_moveStep, 0.0));
                    break;
            case 1: move.setTranslation (Eigen::Vector2d (_moveStep, 0.0));
                    move.setRotation (Eigen::Rotation2Dd (angle));
                    break;
            case 2: move.setTranslation (Eigen::Vector2d (_moveStep, 0.0));
                    move.setRotation (Eigen::Rotation2Dd (-angle));
                    break;
        }

        SE2 simmove ( // odom
            move.translation().x() + gaussian_generator<double> (0.0, xnoise_stdev),
//            move.translation().y() + gaussian_generator<double> (0.0, ynoise_stdev),
//            move.rotation().angle() + gaussian_generator<double> (0.0, pnoise_stdev));
            0.0, 
            move.rotation().angle() == 0.0 ? 0.0 : move.rotation().angle() + gaussian_generator<double>(0.0,pnoise_stdev));

        pose = pose * move;
        simpose = simpose * simmove;

        for (int i = 0; i < _sim->N; i++) {
            SE2 lastpose = _sim->particles[i].path.back();
            SE2 samplemove (
                simmove.translation().x() + gaussian_generator<double> (0.0, xnoise_stdev),
                simmove.translation().y() + gaussian_generator<double> (0.0, ynoise_stdev),
                simmove.rotation().angle() + gaussian_generator<double> (0.0, pnoise_stdev));
            SE2 samplepose = lastpose * samplemove;
            _sim->particles[i].path.push_back (samplepose);
        }

        truePose = pose.toVector();
        simPose = simpose.toVector();

        ofstream mfile;
        mfile.open ("/home/ism/tmp/sim.dat", std::ios::out | std::ios::app);
        mfile   << simpose.translation().x() << " " 
                << simpose.translation().y() << " " 
                << simpose.rotation().angle() << " " 
                << pose.translation().x() << " " 
                << pose.translation().y() << " " 
                << pose.rotation().angle() << std::endl; 
        mfile.close();
    }

    void Simulator::runPF()
    {
        double turn_angle[4] = {90, 90, 90, 100};
        int turn_time[4] = {12, 24, 36, 48};
        int turn_direction[4] = {2,2,2,2};
        int turn_now = 0;

        // t = 0
        double weight = 1.0/(double)N; // initial weight
        for (int i = 0; i < N; i++) {
            Particle p;
            p.weight = weight;
            SE2 firstpose; firstpose.fromVector(robot->truePose); 
            p.path.push_back(firstpose);
            particles.push_back(p);
        }

        ofstream finalposefile;
        finalposefile.open ("/home/ism/tmp/finalpose.dat", ios::out | ios::app);
        ofstream wallfile;
        wallfile.open ("/home/ism/tmp/wall.dat", ios::out | ios::app);
        for (int time = 1; time < MYSLAM::SIM_NUMBER_OF_ITERATIONS; time++)
        {
            if (time == turn_time[turn_now]) {
                robot->sampleMove (turn_direction[turn_now], turn_angle[turn_now] * M_PI/180.0);
                robot->observe();
                turn_now++;
            } else {
                robot->sampleMove();
                robot->observe();
            }

            double x = 0.0;
            double y = 0.0;
            double t = 0.0;

            for (int i = 0; i < N; i++)
            {
                double w = particles[i].weight;
                x += particles[i].path.back().translation().x() * w;
                y += particles[i].path.back().translation().y() * w;
                t += particles[i].path.back().rotation().angle() * w;
            }

//            wallfile << u << " " << v << std::endl;
            wallfile.close();
            finalposefile << x << " " << y << " " << t << std::endl;
            resample();

        }

        finalposefile.close();
        wallfile.close();
    }

    Simulator::Particle::Particle() {
    
    }

    Eigen::Matrix2d* Simulator::Robot::obvJacobian (SE2 pose, Eigen::Vector2d landmark) {
        // sensor model
        // g1 = u cos(t) + v sin(t) - x cos(t) - y sin(t)
        // g2 = -u sin(t) + v cos(t) + x sin(t) - y cos(t)

        // Jacobian Z = | dg1/du    dg1/dv |
        //              | dg2/du    dg2/dv |
        //
        //  dg1/du = cos(t)
        //  dg1/dv = sin(t)
        //  dg2/du = - sin(t)
        //  dg2/dv = cos(t)

//        double angle = pose.rotation().angle();
//        double cost = cos(angle);
//        double sint = sin(angle);
//
//        Eigen::Matrix2d* jac = new Eigen::Matrix2d; 
//        *jac << cost, sint, -sint, cost;
//        return jac;

        double x = pose.translation().x();
        double y = pose.translation().y();
        double t = pose.rotation().angle();
        double xx = landmark[0];
        double xy = landmark[1];

        double dx = xx-x;
        double dy = xy-y;
        double div = dx*dx + dy*dy;

        Eigen::Matrix2d* jac = new Eigen::Matrix2d;
        *jac << dx/sqrt(div), dy/sqrt(div), 
                -dy/div, dx/div;
        return jac;
    }

    void Simulator::resample() {

        // resampling
        std::vector<double> cumdist;
        double cum = 0.0;
        for (int i = 0; i < N; i++) {
            cum += particles[i].weight;
            cumdist.push_back (cum);
        }

        // copy the particles
        std::vector<Particle> p = particles;

        // then clear the original
        particles.clear();

        // fill with resampling result
        double new_weight = 1.0/(double) N;
        for (int i = 0; i < N; i++) {
            double pick = uniform_generator<double>(0,1);

//            auto jt = std::find_if (cumdist.begin(), cumdist.end(), 
//                    [pick](const std::vector<double>& e){ return e >= pick; });

            std::vector<double>::iterator jt;
            jt = std::upper_bound (cumdist.begin(), cumdist.end(), pick);

            if (jt != cumdist.end()) {
                int index = std::distance (cumdist.begin(), jt);
                p[index].weight = new_weight;
                particles.push_back (p[index]);
            }
        } 
    }

    void Simulator::generateLandmark () {
        int nlist[10] = {5,10,20,50,100,200,300,500,700,1000};

        double x, y;

        std::ofstream lmfile;
        lmfile.open ("/home/ism/tmp/lm.dat",std::ios::out|std::ios::app);
        for (int i = 0; i < 10; ++i) {
            lmfile << "double landmark" << nlist[i] << "[" << nlist[i]*2 << "] = {";
            for (int j = 0; j < nlist[i]; j++)
            {
                x = uniform_generator<double>(-12.0,2.0);
                y = uniform_generator<double>(-2.0, 12.0);
                lmfile << x << "," << y << "," << std::endl;
            }
            lmfile << "};" << std::endl << std::endl;
        }
        lmfile.close();
    }

    void Simulator::generateObjects (int n) {

        ofstream objectfile;
        objectfile.open ("/home/ism/tmp/trueobject.dat", std::ios::out | std::ios::app);

        for (int i = 0; i < n; ++i) {
            double x = uniform_generator<double>(-roomwidth + 0.5, roomwidth - 0.5);
            double y = uniform_generator<double>(-roomlength + 0.5, roomlength - 0.5);
            double t = uniform_generator<double>(-M_PI,M_PI);

            Benda b;
            b.id = i;
            SE2 o (x, y, t);
            o = M * o;
            b.pose = o.toVector();
            bendabenda.push_back (b);
            objectfile << b.pose.transpose() << std::endl;
        }

        objectfile.close();
    }

    void Simulator::generateRoom () {

        std::vector<Eigen::Vector2d> data;

        Eigen::Vector2d c1 (0.0, roomlength);
        Eigen::Vector2d p1 (-roomwidth, roomlength);
        Eigen::Vector2d q1 (roomwidth, roomlength);
        data.push_back(c1);
        data.push_back(p1);
        data.push_back(q1);

        Eigen::Vector2d c2 (0.0, -roomlength);
        Eigen::Vector2d p2 (-roomwidth, -roomlength);
        Eigen::Vector2d q2 (roomwidth, -roomlength);
        data.push_back(c2);
        data.push_back(p2);
        data.push_back(q2);

        Eigen::Vector2d c3 (-roomwidth, 0.0);
        Eigen::Vector2d p3 (-roomwidth, -roomlength);
        Eigen::Vector2d q3 (-roomwidth, roomlength);
        data.push_back(c3);
        data.push_back(p3);
        data.push_back(q3);

        Eigen::Vector2d c4 (roomwidth, 0.0);
        Eigen::Vector2d p4 (roomwidth, -roomlength);
        Eigen::Vector2d q4 (roomwidth, roomlength);
        data.push_back(c4);
        data.push_back(p4);
        data.push_back(q4);

        ofstream wfile;
        wfile.open ("/home/ism/tmp/truewall.dat", std::ios::out | std::ios::app);
        for (int i = 0, j = 0; i < 4; i++, j = j + 3)
        {
            Dinding *d = new Dinding;
            d->id = i;
            data[j] = M * data[j];
            d->xx = data[j][0];
            d->xy = data[j][1];
            d->calcMC();
            d->p = M * data[j+1];
            d->q = M * data[j+2];
            struktur.push_back(*d);
            wfile << d->p.transpose() << " " << d->q.transpose() << std::endl;
        }
        wfile.close();
    }

    void Simulator::calculateRMSE (MYSLAM::Graph& graph) {
        std::ofstream rmsefile;
        rmsefile.open ("/home/ism/tmp/rmse.dat",std::ios::out|std::ios::app);

        double dtrans, drot, cumtrans = 0.0, cumrot = 0.0;
        int n = robot->finalpose.size();
        for (int i = 0; i < n; ++i) {
            int id = robot->finalpose[i];

            if (graph._poseMap[id]) {
                double x0 = robot->truePath[i][0];
                double y0 = robot->truePath[i][1];
                double t0 = robot->truePath[i][2];
                double x1 = graph._poseMap[id]->_pose[0];
                double y1 = graph._poseMap[id]->_pose[1];
                double t1 = graph._poseMap[id]->_pose[2];

                dtrans = (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1);
                drot = normalize_theta (t0-t1); 
                drot = drot*drot;

                cumtrans += dtrans;
                cumrot += drot;
            }
        }

        double rmsetrans = sqrt (cumtrans/n);
        double rmserot = sqrt (cumrot/n);

        rmsefile << rmsetrans << " " << normalize_angle (rmserot) << std::endl;
        rmsefile.close();
    }

    void Simulator::Dinding::calcMC() {
        m = -xx/xy;
        c = (xx*xx + xy*xy)/xy;
    }
}

namespace MYSLAM {
    double objects[30] = {
        -9.56671, 4.63100, -45.0*M_PI/180.0,
        -8.04104, 6.43665, -10.0*M_PI/180.0,
        -4.30076, 9.72672, -135.0*M_PI/180.0,
        1.33112, 6.77002, -135.0*M_PI/180.0,
        6.10102, 2.96141, 170.0*M_PI/180.0,
        -5.17938, 6.36944, 0.0,
        3.81101, -0.621619, M_PI,
        -6.56705, -1.34290, 30.0*M_PI/180.0,
        -9.14475, 0.883526, 45.0*M_PI/180.0,
        0.851700, -4.07258, 135.0*M_PI/180.0};
}
