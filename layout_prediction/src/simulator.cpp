#include <random>
#include <iostream>
#include <fstream>

#include "layout_prediction/simulator.h"

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
}

void Simulator::Robot::move()
{
    sensedData.clear();

    const double xdis = 0.5;
    const double ydis = 0.5;
    const double pdis = 1.0 * M_PI/180;

    double xmov = uniform_generator<double> (-xdis, xdis);
    double ymov = uniform_generator<double> (-ydis, ydis);
    double pmov = uniform_generator<double> (-pdis, pdis);

    SE2 trueMove (xmov, ymov, pmov);
    SE2 tmpMove = trueMove * *truePose;
    if (tmpMove.translation().x() > -20 && tmpMove.translation().x() < 50 &&
        tmpMove.translation().y() > -20 && tmpMove.translation().y() < 70)
        *truePose = tmpMove;
    else
        *truePose = trueMove.inverse() * *truePose;

//    ofstream truef;
//    truef.open ("/home/ism/tmp/true.dat", ios::out | ios::app);
//    truef << truePose->toVector().transpose() << std::endl;
//    truef.close();

    double xnoise = gaussian_generator<double>(0.0, xNoise);
    double ynoise = gaussian_generator<double>(0.0, yNoise);
    double pnoise = gaussian_generator<double>(0.0, pNoise);

    SE2 simMove (xnoise, ynoise, pnoise);
    *simPose = simMove * *truePose;

//    ofstream simf;
//    simf.open ("/home/ism/tmp/sim.dat", ios::out | ios::app);
//    simf << simPose->toVector().transpose() << std::endl;
//    simf.close();
}

void Simulator::Robot::sense()
{
    double x = simPose->translation().x();
    double y = simPose->translation().y();
    double phi = simPose->rotation().angle();
    double cosphi = cos(phi);
    double sinphi = sin(phi);

    // sensor model
    for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
    {
        double rho = (*it).rho - x*cosphi - y*sinphi;
        double theta = (*it).theta - phi;

        if (abs(theta) <= FOV && rho <= RANGE)
            sensedData.push_back (&(*it));
    }
}

Simulator::Simulator()
{
    double grads[8] = {-.5, 2, -.5, 2, -.5, 2, -.5, 2};
    double intercepts[8] = {-12.0, 30.0, 80.0, -30.0, 40.0, 15.0, 10.0, -30.0};
    double points[18] = {7.2, -15.6, -16.8, -3.6, 20.0, 70.0, 44.0, 58.0, 28.0, 26.0, 10.0, 35.0, -2.0, 11.0, 16.0, 2.0, 7.2, -15.6};

    for (int i = 0; i < NUM_OF_WALLS; i++)
    {
        int j = 0;
        Dinding *d = new Dinding;
        d->id = i;
        d->m = grads[i];
        d->c = intercepts[i];
        double rho = abs(d->c)/(d->m*d->m+1);
        double theta = atan2 (-d->m*d->c/(d->m*d->m+1),d->c/(d->m*d->m+1));
        d->rho = rho;
        d->theta = theta;
        Vector2d p (points[j], points[j+1]);
        Vector2d q (points[j+2], points[j+3]);
        d->p = p;
        d->q = q;
        struktur.push_back(*d);
        j += 2;
    }

    robot = new Robot(this);
}

Simulator::Robot* Simulator::getNextState()
{
    robot->move();
    robot->sense();
    return robot;
}

