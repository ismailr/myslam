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

    void Simulator::Robot::moveStraight() {
        SE2 trupose;
        trupose.fromVector (truePose);  

        SE2 trumove (_moveStep, 0.0, 0.0);
        trupose = trupose * trumove;
        truePose = trupose.toVector();

        SE2 simpose;
        simpose.fromVector(simPose);

        SE2 simmove (
                trumove.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
                trumove.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
                trumove.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
        simpose = simpose * simmove;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::move()
    {
        sensedData.clear();
        sensedDinding.clear();

        double xtrue = truePose[0];
        double ytrue = truePose[1];
        double ptrue = truePose[2];
        double dx = xtrue;
        double dy = ytrue;
        xtrue += _moveStep * cos (ptrue);
        ytrue += _moveStep * sin (ptrue);
        dx = xtrue - dx;
        dy = ytrue - dy;
        double d = sqrt ((dx*dx) + (dy*dy));

        truePose << xtrue, ytrue, ptrue;

        double noise = gaussian_generator<double>(0.0, xnoise_stdev);
        d = d + noise;

        double xsim = simPose[0];
        double ysim = simPose[1];
        double psim = simPose[2];
        xsim = xsim + d * cos(psim);
        ysim = ysim + d * sin(psim);

        simPose << xsim, ysim, psim;
    }

    void Simulator::Robot::turn (int direction, double angle)
    {
        sensedData.clear();
        sensedDinding.clear();

        direction == 0 ? angle = angle : angle = -angle;
        SE2 truturn (_moveStep, 0.0, angle);

        SE2 trupose;
        trupose.fromVector (truePose);
        trupose = trupose * truturn;
        truePose = trupose.toVector();

        SE2 simpose;
        simpose.fromVector (simPose);

        SE2 simturn (
                truturn.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
                truturn.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
                truturn.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
        simpose = simpose * simturn;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::sense()
    {
        double x = simPose[0];
        double y = simPose[1];
        double t = simPose[2];
        double cost = cos(t);
        double sint = sin(t);

        // sensor model
        for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
        {
            double xxnoise = gaussian_generator<double>(0.0, wnoise_stdev);
            double xynoise = gaussian_generator<double>(0.0, wnoise_stdev);

            double& xx = (*it).xx;
            double& xy = (*it).xy;

            double xx_ = xx*cost + xy*sint - x*cost - y*sint + xxnoise;
            double xy_ = -xx*sint + xy*cost + x*sint - y*cost + xynoise;

            Eigen::Vector2d data (xx_, xy_);

            double r = sqrt (xx_*xx_ + xy_*xy_);
            double angle = normalize_theta (atan2 (xy_,xx_));

            if (r <= RANGE)
            {
//                if ((angle >= 0.0 && angle < M_PI/2) || (angle > 3*M_PI/2 && angle <= 2*M_PI))
//                {
                    sensedDinding.push_back (&(*it));
                    sensedData.push_back (data);
//                }
            }
        }
    }

    void Simulator::Robot::sensePoint()
    {
        double x = simPose[0];
        double y = simPose[1];
        double p = simPose[2];
        double cosp = cos(p);
        double sinp = sin(p);

        // sensor model
        for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
        {
            double& lx = (*it).m;
            double& ly = (*it).c;

            double xxnoise = gaussian_generator<double>(0.0, wnoise_stdev);
            double xynoise = gaussian_generator<double>(0.0, wnoise_stdev);

            double x_ = (lx - x) * cosp;
            double y_ = (ly - y) * sinp;

            Eigen::Vector2d data (x_,y_);

            double r = sqrt (x_*x_ + y_*y_);
            double angle = normalize_theta (atan2 (y_,x_));

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

    Simulator::Simulator()
    {
        const int N = 100;
//        double grads[4] = {2, -.5, 2, -.5};
//        double intercepts[4] = {25.0, 8.0, -10.0, -5.0};
//        double x[5] = {-12, -6.8, 7.2, 2, -12};
//        double y[5] = {1, 11.4, 4.4, -6, 1};

        std::vector<double> grads (N); 
        std::vector<double> intercepts (N); 
        std::vector<double> x(N + 1); 
        std::vector<double> y(N + 1); 

        for (int i = 0; i < N; i++)
        {
            grads[i] = uniform_generator<double>(-5.0,5.0);
            intercepts[i] = uniform_generator<double>(-100.0, 100.0);
            x[i] = uniform_generator<double>(-5.0,5.0);
            y[i] = uniform_generator<double>(-5.0,5.0);
        }

        x.push_back(x[0]);
        y.push_back(y[0]);

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
        robot->moveStraight();
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

        Eigen::Vector3d lastSimPose;
        Pose::Ptr lastPose;

        double turn_angle[4] = {90, 90, 90, 100};
        int turn_time[4] = {12, 24, 36, 48};
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

            if (frame % 10 == 0)
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
                robot->moveStraight();
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

    void Simulator::runPoint()
    {
        MYSLAM::Graph graph;
        Optimizer o(graph);

        ofstream mfile;
        mfile.open ("/home/ism/tmp/sim.dat", ios::out | ios::app);

        std::vector<Dinding*> dindings;
        std::vector<Point::Ptr> points;

        Eigen::Vector3d lastSimPose;
        Pose::Ptr lastPose;

        double turn_angle[4] = {120.0, 100.0, 50.0, 135.0};
        int turn_time[4] = {15, 18, 25, 27};
        int turn_direction[4] = {0, 0, 1, 0};
        int turn_now = 0;

        for (int frame = 1; frame < MYSLAM::SIM_NUMBER_OF_ITERATIONS; frame++)
        {
            // poses
            Eigen::Vector3d t = robot->truePose;
            Eigen::Vector3d s = robot->simPose;

            mfile   << s[0] << " " << s[1] << " " << s[2] << " " << t[0] << " " << t[1] << " " << t[2] << std::endl;

            Pose::Ptr pose (new Pose);
            if (frame == 1)
            {
                pose->_pose = s;
                graph._poseMap [pose->_id] = pose;
                graph._activePoses.push_back (pose->_id);
                lastSimPose = s;
                lastPose = pose;
                getNextState();
                continue;
            }

            Eigen::Vector3d delta = s - lastSimPose;
            double dr = sqrt (delta[0]*delta[0] + delta[1]*delta[1]);
            double current_heading = normalize_angle (lastPose->_pose[2] + delta[2]);
            double dx = dr * cos (current_heading);
            double dy = dr * sin (current_heading);
            Eigen::Vector3d incr (dx, dy, delta[2]);

            pose->_pose = lastPose->_pose + incr;
            graph._poseMap [pose->_id] = pose;
            graph._activePoses.push_back (pose->_id);

            for (size_t i = 0; i < robot->sensedDinding.size(); i++)
            {
                // asosiasi data
                Point::Ptr w = NULL;
                Dinding* d = robot->sensedDinding[i];

                if (dindings.size() == 0)
                {
                    Point::Ptr point (new Point);
                    w = point;
                    dindings.push_back (d);
                    points.push_back (w);

                } else {
                    for (int j = 0; j < dindings.size(); j++)
                    {
                        if (d->id == dindings[j]->id)
                        {
                            w = points[j];
                            break;
                        }
                        else if (j == dindings.size() - 1)
                        {
                            Point::Ptr point (new Point);
                            w = point;
                            dindings.push_back (robot->sensedDinding[i]);
                            points.push_back (w);
                            break;
                        }
                    }
                }

                double& x_ = robot->sensedData[i][0];
                double& y_ = robot->sensedData[i][1];

                double x = pose->_pose[0];
                double y = pose->_pose[1];
                double t = pose->_pose[2];


                w->x = x + x_*cos(t);
                w->y = y + y_*sin(t);

                graph._pointMap [w->_id] = w;
                graph._activePoints.insert (w->_id);

                std::tuple<int, int> e (pose->_id, w->_id);
                graph._posePointMap [e] = robot->sensedData[i];
                graph._activeEdges.push_back (e);
            }

            if (frame % 2 == 0)
            {
                o.localOptimize();
            }

//            getNextState();
            lastSimPose = s;
            lastPose = pose;


            if (frame == turn_time[turn_now]) {
                robot->turn (turn_direction[turn_now], turn_angle[turn_now] * M_PI/180.0);
                robot->sensePoint();
                turn_now++;
            } else {
                robot->move();
                robot->sensePoint();

            }
        }

//        o.globalOptimize();

        mfile.close();

        ofstream wfile;
        wfile.open ("/home/ism/tmp/point.dat", std::ios::out | std::ios::app);
        for (std::map<int, Point::Ptr>::iterator it = graph._pointMap.begin();
                it != graph._pointMap.end(); it++)
        {
            wfile << it->second->x << " " << it->second->y << std::endl;
        }
//        wfile << std::endl;

//        for (int i = 0; i < struktur.size(); i++)
//        {
//            wfile << struktur[i].id << " " << struktur[i].xx << " " << struktur[i].xy << std::endl;
//        }
//        wfile << std::endl;
        wfile.close();

    }
}
