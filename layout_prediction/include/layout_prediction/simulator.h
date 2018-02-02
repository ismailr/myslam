#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <utility>

#include "se2.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/settings.h"

#include <Eigen/Core>

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

        class Particle
        {
            public:
                Particle();

                std::vector<SE2> path;
                std::vector<std::tuple<int, Dinding ,Eigen::Matrix2d> > landmarks;
                double weight;

            private:
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
                void turn(int direction, double angle);
                void sense();

                void sampleMove(int direction = 0, double angle = 0.0); // particle filter
                void observe(); // particle filter

                Eigen::Matrix2d* obvJacobian (SE2, Eigen::Vector2d);

                double _moveStep;

            private:
                Simulator* _sim;
                const double xnoise_stdev = 5e-2, ynoise_stdev = 1e-2, pnoise_stdev  = 2.0 * M_PI/180.; 
                const double wnoise_stdev = 5e-2;
                const double RANGE = 3.0;
        };

        std::vector<Dinding> struktur;
        Robot *robot;
        std::vector<Particle> particles;
        int N = MYSLAM::PF_NUMBER_OF_PARTICLES; // Num of particles

        void getNextState();
        void getNextStatePF();

        std::vector<Dinding> getStruktur () const { return struktur; };

        void run();
        void runPF();

        void resample();

        private:
    };
}

#endif
