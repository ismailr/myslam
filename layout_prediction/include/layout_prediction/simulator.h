#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include "se2.h"

using namespace g2o;

class Simulator
{
    public:
        Simulator();

    struct Dinding
    {
        int id;
        double m, c, rho, theta;
        Eigen::Vector2d p;
        Eigen::Vector2d q;
    };

    class Robot
    {
        public:
            Robot(Simulator*);

            SE2 *truePose;
            SE2 *simPose;
            std::vector<Dinding*> sensedData;

            void move();
            void sense();

        private:
            Simulator* _sim;
            const double xNoise = 0.5, yNoise = 0.5;
            const double pNoise  = 5.0 * M_PI/180.0;
            const double RANGE = 20.0;
            const double FOV = M_PI/2;
    };

    std::vector<Dinding> struktur;
    Robot *robot;

    Robot* getNextState();
    std::vector<Dinding> getStruktur () const { return struktur; };

    private:
    const int NUM_OF_WALLS = 8;
};

#endif
