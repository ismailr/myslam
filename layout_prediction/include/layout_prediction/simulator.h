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

                Eigen::Vector3d truePose;
                Eigen::Vector3d simPose;
                std::vector<Dinding*> sensedDinding;
                std::vector<Eigen::Vector2d> sensedData;

                void move();
                void sense();

            private:
                Simulator* _sim;
                const double xnoise_var = 0.01, ynoise_var = 0.01;
                const double pnoise_var  = 0.01 * M_PI/180.0;
                const double wnoise_var = 0.1;
                const double RANGE = 15.0;
        };

        std::vector<Dinding> struktur;
        Robot *robot;

        void getNextState();
        std::vector<Dinding> getStruktur () const { return struktur; };

        void run();
        void runsimple();
        void runsimple2();

        private:
        const int NUM_OF_WALLS = 8;
    };
}

#endif
