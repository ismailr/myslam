#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include "se2.h"

using namespace g2o;

namespace MYSLAM {
    class Simulator
    {
        public:
            Simulator();

        struct Dinding
        {
            int id;
            double m, c, xx, xy;
            Eigen::Vector2d p;
            Eigen::Vector2d q;
        };

        class Robot
        {
            public:
                Robot(Simulator*);

                SE2 *truePose;
                SE2 *simPose;
                std::vector<Dinding*> sensedDinding;
                std::vector<Eigen::Vector2d> sensedData;

                void move();
                void sense();

            private:
                Simulator* _sim;
                const double xNoise = 0.01, yNoise = 0.01;
                const double pNoise  = 0.01 * M_PI/180.0;
                const double RANGE = 2.0;
        };

        std::vector<Dinding> struktur;
        Robot *robot;

        void getNextState();
        std::vector<Dinding> getStruktur () const { return struktur; };

        void run();

        private:
        const int NUM_OF_WALLS = 8;
    };
}

#endif
