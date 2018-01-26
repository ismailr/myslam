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
                void moveStraight();
                void turn(int direction, double angle);
                void sense();
                void sensePoint();

                double _moveStep;

            private:
                Simulator* _sim;
                const double xnoise_stdev = 5e-2, ynoise_stdev = 1e-2, pnoise_stdev  = 2.0 * M_PI/180.; 
                const double wnoise_stdev = 5e-2;
                const double RANGE = 100.0;
        };

        std::vector<Dinding> struktur;
        Robot *robot;

        void getNextState();
        std::vector<Dinding> getStruktur () const { return struktur; };

        void run();
        void runPoint();

        private:
    };
}

#endif
