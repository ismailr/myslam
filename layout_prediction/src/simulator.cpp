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
#include "layout_prediction/point.h"
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
    Simulator::Robot::Robot(Simulator* sim) 
        : _sim (sim), _moveStep (0.5)
    {
        truePose.setZero(); 
        simPose.setZero();
        truePose[2] = 3.0;
        simPose[2] = truePose[2];
//        sense();
    }

    void Simulator::Robot::move() {
        SE2 pose;
        pose.fromVector (truePose);  

        SE2 move (_moveStep, 0.0, 0.0);
        pose = pose * move;
        truePose = pose.toVector();

        SE2 simpose;
        simpose.fromVector(simPose);

        SE2 simmove (
                move.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
                move.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
                move.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
        simpose = simpose * simmove;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::turn (int direction, double angle)
    {
        direction == 0 ? angle = angle : angle = -angle;
        SE2 turn (_moveStep, 0.0, angle);

        SE2 pose;
        pose.fromVector (truePose);
        pose = pose * turn;
        truePose = pose.toVector();

        SE2 simpose;
        simpose.fromVector (simPose);

        SE2 simturn (
                turn.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
                turn.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
                turn.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
        simpose = simpose * simturn;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::sense()
    {
        sensedData.clear();
        sensedDinding.clear();

        // sensor model
        for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
        {
            double unoise = gaussian_generator<double>(0.0, wnoise_stdev);
            double vnoise = gaussian_generator<double>(0.0, wnoise_stdev);

            double u = (*it).xx;
            double v = (*it).xy;

            SE2 simpose; simpose.fromVector (simPose);

            Eigen::Vector2d xx (u, v);
            Eigen::Vector2d xx_ = simpose.inverse() * xx;

            Eigen::Vector2d data (xx_[0] + unoise, xx_[1] + vnoise);

            double r = sqrt (xx_[0]*xx_[0] + xx_[1]*xx_[1]);
            double angle = normalize_theta (atan2 (xx_[1],xx_[0]));

            if (r <= RANGE)
            {
                if ((angle >= 0.0 && angle < M_PI/2) || (angle > 3*M_PI/2 && angle <= 2*M_PI))
                {
                    sensedDinding.push_back (&(*it));
                    sensedData.push_back (data);
                }
            }
        }
    }

    void Simulator::Robot::update ()
    {

    }

    void Simulator::Robot::observe()
    {
        // observation noise
        Eigen::Matrix2d R;
        double var = wnoise_stdev * wnoise_stdev;
        R << var, 0, 0, var;
                                
        for (int i = 0; i < _sim->N; i++)
        {
            // current pose
            SE2 pose = _sim->particles[i].path.back(); 
            Eigen::Matrix2d G = *obvJacobian (pose);


            // iterate over walls
            for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
            {
                double unoise = gaussian_generator<double>(0.0, wnoise_stdev);
                double vnoise = gaussian_generator<double>(0.0, wnoise_stdev);

                Eigen::Vector2d globalWall ((*it).xx,(*it).xy);
                Eigen::Vector2d localWall = pose.inverse() * globalWall;

                localWall[0] += unoise;
                localWall[1] += vnoise;

                double r = sqrt (localWall[0]*localWall[0] + localWall[1]*localWall[1]);
                double angle = normalize_angle (atan2 (localWall[1],localWall[0]));

                if (r <= RANGE)
                {
                    if ((angle >= 0.0 && angle < M_PI/2) || (angle > 3*M_PI/2 && angle <= 2*M_PI)) //landmark observed
                    {
                        bool new_landmark = false;
                        
                        if (_sim->particles[i].landmarks.size() == 0) { // first observed
                            new_landmark = true;
                        } else {
                            // find in landmarks whether this Dinding already observed
                            auto jt = find_if ( _sim->particles[i].landmarks.begin(),
                                                _sim->particles[i].landmarks.end(),
                                                [&it](const std::tuple<int, Dinding, Eigen::Matrix2d>& e)
                                                { return std::get<0>(e) == (*it).id; });

                            if (jt == _sim->particles[i].landmarks.end()) { new_landmark = true; } else {
                                // update landmark
                                Eigen::Matrix2d S = std::get<2>(*jt);
                                Eigen::Matrix2d Z = G * S * G.transpose() + R;
                                Eigen::Matrix2d K = S * G * Z.inverse();
                                Eigen::Vector2d oldEstimate;
                                oldEstimate[0] = std::get<1>(*jt).xx;
                                oldEstimate[1] = std::get<1>(*jt).xy;
                                Eigen::Vector2d newEstimate = oldEstimate + K * (localWall - pose.inverse() * oldEstimate);
                                Eigen::Matrix2d newCov = (Eigen::Matrix2d::Identity() - K * G) * S;
                                std::get<1>(*jt).xx = newEstimate[0];
                                std::get<1>(*jt).xy = newEstimate[1];
                                std::get<2>(*jt) = newCov;
                            }
                        }

                        if (new_landmark) {
                            //inverse sensor model
                            Eigen::Vector2d observedWall = pose * localWall;
                            Dinding d;
                            d.id = (*it).id;
                            d.xx = observedWall[0];
                            d.xy = observedWall[1];
                            Eigen::Matrix2d jac = *obvJacobian (pose);
                            jac = jac.transpose() * R.inverse() * jac;
                            std::tuple<int, Dinding, Eigen::Matrix2d> l 
                                = std::make_tuple (d.id, d, jac.inverse());
                            _sim->particles[i].landmarks.push_back (l);
                        }
                    }
                }
            }
        }
    }

    Simulator::Simulator()
    {
        const int N = 4;
        double grads[4] = {2, -.5, 2, -.5};
        double intercepts[4] = {25.0, 8.0, -10.0, -5.0};
        double x[5] = {-12, -6.8, 7.2, 2, -12};
        double y[5] = {1, 11.4, 4.4, -6, 1};

//        std::vector<double> grads (N); 
//        std::vector<double> intercepts (N); 
//        std::vector<double> x(N + 1); 
//        std::vector<double> y(N + 1); 
//
//        for (int i = 0; i < N; i++)
//        {
//            grads[i] = uniform_generator<double>(-5.0,5.0);
//            intercepts[i] = uniform_generator<double>(-100.0, 100.0);
//            x[i] = uniform_generator<double>(-5.0,5.0);
//            y[i] = uniform_generator<double>(-5.0,5.0);
//        }
//
//        x.push_back(x[0]);
//        y.push_back(y[0]);

        for (int i = 0; i < N; i++)
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

    void Simulator::getNextStatePF()
    {
        robot->sampleMove();
        robot->update();
    }


    void Simulator::run()
    {
        MYSLAM::Graph graph;
        Optimizer o(graph);

        ofstream mfile;
        mfile.open ("/home/ism/tmp/sim.dat", ios::out | ios::app);

        std::vector<Dinding*> dindings;
        std::vector<Wall::Ptr> walls;

        Eigen::Vector3d lastSimPose;
        Pose::Ptr lastPose;

        double turn_angle[4] = {0, 125, 100, 90};
        int turn_time[4] = {12, 20, 40, 48};
        int turn_direction[4] = {1,1,1,1};
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
        getNextState();

        for (int frame = 1; frame < MYSLAM::SIM_NUMBER_OF_ITERATIONS; frame++)
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

            for (size_t i = 0; i < robot->sensedDinding.size(); i++)
            {
                // asosiasi data
                Wall::Ptr w = NULL;
                Dinding* d = robot->sensedDinding[i];

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
                            break;
                        }
                        else if (j == dindings.size() - 1)
                        {
                            Wall::Ptr wall (new Wall);
                            w = wall;
                            w->_line.p = d->p;
                            w->_line.q = d->q;
                            w->initParams();
                            dindings.push_back (robot->sensedDinding[i]);
                            walls.push_back (w);
                            break;
                        }
                    }
                }

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

                graph._wallMap [w->_id] = w;
                graph._activeWalls.insert (w->_id);

                std::tuple<int, int> e (pose->_id, w->_id);
                graph._poseWallMap [e] = robot->sensedData[i];
                graph._activeEdges.push_back (e);
            }

            if (frame % 2 == 0)
            {
                o.localOptimize();
            }

            lastSimPose = s;
            lastPose = pose;

            if (frame == turn_time[turn_now]) {
                robot->turn (turn_direction[turn_now], turn_angle[turn_now] * M_PI/180.0);
                robot->sense();
                turn_now++;
            } else {
                robot->move();
                robot->sense();
            }
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
//        wfile << std::endl;

//        for (int i = 0; i < struktur.size(); i++)
//        {
//            wfile << struktur[i].id << " " << struktur[i].xx << " " << struktur[i].xy << std::endl;
//        }
//        wfile << std::endl;
        wfile.close();

    }

    void Simulator::Robot::sampleMove ()
    {
        SE2 trueMove (_moveStep, 0, 0);

        for (int i = 0; i < _sim->N; i++) {

            SE2 lastPose = _sim->particles[i].path.back();
            SE2 simMove (
                    trueMove.translation().x() + gaussian_generator<double> (0.0, xnoise_stdev),
                    trueMove.translation().y() + gaussian_generator<double> (0.0, ynoise_stdev),
                    trueMove.rotation().angle() + gaussian_generator<double> (0.0, pnoise_stdev));
            SE2 currentPose = lastPose * simMove;
            _sim->particles[i].path.push_back (currentPose);
        }
    }

    void Simulator::Robot::sampleTurn ()
    {
        SE2 trueMove (_moveStep, 0, 0);

        for (int i = 0; i < _sim->N; i++) {

            SE2 lastPose = _sim->particles[i].path.back();
            SE2 simMove (
                    trueMove.translation().x() + gaussian_generator<double> (0.0, xnoise_stdev),
                    trueMove.translation().y() + gaussian_generator<double> (0.0, ynoise_stdev),
                    trueMove.rotation().angle() + gaussian_generator<double> (0.0, pnoise_stdev));
            SE2 currentPose = lastPose * simMove;
            _sim->particles[i].path.push_back (currentPose);
        }
    }

    void Simulator::runPF()
    {
        ofstream mfile;
        mfile.open ("/home/ism/tmp/sim.dat", ios::out | ios::app);

        std::vector<Dinding*> dindings;
        std::vector<Wall::Ptr> walls;

        double turn_angle[4] = {90, 90, 90, 100};
        int turn_time[4] = {12, 24, 36, 48};
        int turn_direction[4] = {1,1,1,1};
        int turn_now = 0;

        // t = 0
        double weight = 1/N; // initial weight
        for (int i = 0; i < N; i++) {
            Particle p;
            p.weight = weight;
            SE2 firstpose;
            p.path.push_back(firstpose);
            particles.push_back(p);
            robot->truePose = firstpose.toVector();
        }

        getNextStatePF();

        for (int time = 1; time < MYSLAM::SIM_NUMBER_OF_ITERATIONS; time++)
        {
            // sampling new pose
            robot->sampleMove();
            robot->observe();
        }

        mfile.close();
    }

    Simulator::Particle::Particle() {
    
    }

    Eigen::Matrix2d* Simulator::Robot::obvJacobian (SE2 pose) {
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

        double angle = pose.rotation().angle();
        double cost = cos(angle);
        double sint = sin(angle);

        Eigen::Matrix2d* jac = new Eigen::Matrix2d; 
        *jac << cost, sint, -sint, cost;
        return jac;
    }
}
