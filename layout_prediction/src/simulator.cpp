#include <random>
#include <iostream>
#include <fstream>
#include <limits>

#include "layout_prediction/simulator.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/wall_measurement.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/optimizer.h"

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
}

namespace MYSLAM {
    Simulator::Robot::Robot(Simulator* sim) : _sim (sim) 
    {
        truePose = new SE2 (0.0, 0.0, 0.0);
        simPose = new SE2 (0.0, 0.0, 0.0);
        sense();
    }

    void Simulator::Robot::move()
    {
        sensedData.clear();

        const double xdis = 1.0;
        const double ydis = 1.0;
        const double pdis = 1.0 * M_PI/180.0;

        double xmov = uniform_generator<double> (-xdis, xdis);
        double ymov = uniform_generator<double> (-ydis, ydis);
        double pmov = uniform_generator<double> (-pdis, pdis);

        SE2 trueMove (xmov, ymov, pmov);
        SE2 tmpMove = trueMove * *truePose;
        if (tmpMove.translation().x() > -20 && tmpMove.translation().x() < 50 &&
            tmpMove.translation().y() > -20 && tmpMove.translation().y() < 70)
            *truePose = tmpMove;
        else
        {
            trueMove = trueMove.inverse();
            *truePose = trueMove * *truePose;
        }

        double xnoise = gaussian_generator<double>(0.0, xNoise);
        double ynoise = gaussian_generator<double>(0.0, yNoise);
        double pnoise = gaussian_generator<double>(0.0, pNoise);

        SE2 noise (xnoise, ynoise, pnoise);
        SE2 simMove = noise * trueMove;
        *simPose = simMove * *simPose;
    }

    void Simulator::Robot::sense()
    {

        double x = simPose->translation().x();
        double y = simPose->translation().y();
        double p = simPose->rotation().angle();
        double cosp = cos(p);
        double sinp = sin(p);

        // sensor model
        for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
        {
            double& m = (*it).m;
            double& c = (*it).c;

            double m_ = (-sinp + m * cosp)/(cosp + m * sinp);
            double c_ = (c - y + m * x)/(cosp + m * sinp);

            double xxnoise = gaussian_generator<double>(0.0, 1.0);
            double xynoise = gaussian_generator<double>(0.0, 1.0);

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
        double grads[8] = {-.5, 2, -.5, 2, -.5, 2, -.5, 2};
        double intercepts[8] = {-12.0, 30.0, 80.0, -30.0, 40.0, 15.0, 10.0, -30.0};
        double x[9] = {7.2,-16.8,20.0,44.0,28.0,10.0,-2.0,16.0,7.2};
        double y[9] = {-15.6,-3.6,70.0,58.0,26.0,35.0,11.0,2.0,-15.6};

        for (int i = 0; i < NUM_OF_WALLS; i++)
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
        mfile.open ("/home/ism/tmp/true.dat", ios::out | ios::app);

        Eigen::Matrix<double, 3, 3> poseInfMatrix;
        poseInfMatrix.setIdentity();
    //    poseInf << 1, 0, 0, 0, 1, 0, 0, 0, 0.0;
        Eigen::Matrix<double, 2, 2> wallInfMatrix;
        wallInfMatrix.setIdentity();
    //    wallInf << 0.1, 0, 0, 0.1;

        std::vector<Dinding*> dindings;
        std::vector<Wall::Ptr> walls;

        for (int frame = 1; frame < 201; frame++)
        {
            // poses
            SE2 *t = robot->truePose;
            SE2 *s = robot->simPose;

            Pose::Ptr pose (new Pose);
            pose->_pose = s->toVector();
            pose->_poseByModel = t->toVector();
            graph._poseMap [pose->_id] = pose;
            graph._activePoses.push_back (pose->_id);

            for (size_t i = 0; i < robot->sensedDinding.size(); i++)
            {
                // asosiasi data
                Wall::Ptr w;
                Dinding* d = robot->sensedDinding[i];
                for (int i = 0; i < dindings.size(); i++)
                {
                    if (d->id == dindings[i]->id)
                    {
                        w = walls[i];
                        break;
                    }
                    else if (i == dindings.size() - 1)
                    {
                        Wall::Ptr wall (new Wall);
                        w = wall;
                        dindings.push_back (robot->sensedDinding[i]);
                        walls.push_back (w);
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
            }

            if (frame % 5)
                o.localOptimize();

            getNextState();
        }

        mfile.close();
    }
}
