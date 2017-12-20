#include <random>
#include <iostream>
#include <fstream>

#include "layout_prediction/simulator.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/graph.h"

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
    bool init = true;
    bool firstOpt = true;
    Graph2 graph;

    ofstream mfile;
    mfile.open ("/home/ism/tmp/true.dat", ios::out | ios::app);

    Eigen::Matrix<double, 3, 3> poseInf;
    poseInf.setIdentity();
//    poseInf << 1, 0, 0, 0, 1, 0, 0, 0, 0.0;
    Eigen::Matrix<double, 2, 2> wallInf;
    wallInf.setIdentity();
//    wallInf << 0.1, 0, 0, 0.1;

    std::vector<Dinding*> dinding;
    std::vector<Wall3::Ptr> wall;
    Pose2::Ptr lastPose; 

    for (int frame = 1; frame < 201; frame++)
    {
        SE2 t = *(robot->truePose);
        SE2 s = *(robot->simPose);

        mfile << t.translation().x() << " " << t.translation().y() << " " << t.rotation().angle() << " " 
            << s.translation().x() << " " << s.translation().y() << " " << s.rotation().angle() << std::endl;

        Pose2::Ptr pose = graph.createPoseWithId();
        pose->setEstimate(s);

        if (!init)
        {
            PoseMeasurement2::Ptr pm = graph.createPoseMeasurement ();
            pm->vertices()[0] = lastPose.get();
            pm->vertices()[1] = pose.get();
            pm->setMeasurement (lastPose->estimate().inverse() * pose->estimate());
            pm->information() = poseInf;
        }

        double x = pose->estimate().translation().x();
        double y = pose->estimate().translation().y();
        double p = pose->estimate().rotation().angle();
        double cosp = cos(p);
        double sinp = sin(p);

        for (int i = 0; i < robot->sensedData.size(); i++)
        {
            // inverse measurement
            double& xx_ = robot->sensedData[i].x();
            double& xy_ = robot->sensedData[i].y();
            double wData[2] = {xx_,xy_};

            double xx, xy, m_, c_, m, c;
            if (xy_ != 0)
            {
                m_ = -xx_/xy_;
                c_ = (xx_*xx_ + xy_*xy_)/xy_;
                m = (sinp + m_ * cosp)/(cosp - m_ * sinp);
                c = -m * x + y + c_/(cosp - m_ * sinp);
                xx = -m * c/(m*m+1);
                xy = c/(m*m+1);

            } else {
                m_ = numeric_limits<double>::infinity();
                c_ = numeric_limits<double>::infinity();
                m = (sinp + m_ * cosp)/(cosp - m_ * sinp);
                c = -m * x + y + c_/(cosp - m_ * sinp);
                xx = -m * c/(m*m+1);
                xy = c/(m*m+1);
            }

            Wall3::Ptr w = graph.createWall3WithId();

            // asosiasi data
            bool hit = false;
            for (int j = 0; j < dinding.size(); j++)
            {
                if (robot->sensedDinding[i]->id == dinding[j]->id)
                {
                    w = wall[j];
                    hit = true;
                    break;
                }
            }

            if (!hit) {
                graph.registerWall3 (w);
                double data[2] = {xx, xy};
                w->setEstimateDataImpl (data);
                dinding.push_back (robot->sensedDinding[i]);
                wall.push_back (w);
            }

            WallMeasurement3::Ptr wm = graph.createWallMeasurement3();
            wm->vertices()[0] = pose.get();
            wm->vertices()[1] = w.get();
            wm->setMeasurementData (wData);
            wm->information() = wallInf;
        }

        lastPose = pose;

        if (frame % 5 == 0)
        {
            graph.localOptimize (firstOpt);
            dinding.clear();
            wall.clear();
            firstOpt = false;
        }

        init = false;
        getNextState();
    }

//    graph.localOptimize (firstOpt);
    mfile.close();
}
