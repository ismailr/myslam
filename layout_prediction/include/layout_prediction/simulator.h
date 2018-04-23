#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <Eigen/Core>
#include <g2o/types/slam2d/se2.h>

#include "layout_prediction/settings.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/graph.h"

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

            void calcMC();
        };

        struct Benda
        {
            int id;
            int classid;
            Eigen::Vector3d pose;

            Benda ();
        };

        class Robot
        {
            public:
                Robot(Simulator*);

                Eigen::Vector3d truePose;
                Eigen::Vector3d simPose;
                std::vector<Dinding*> sensedDinding;
                std::vector<Eigen::Vector2d> sensedData;
                std::vector<Benda*> sensedBenda;
                std::vector<Eigen::Vector3d> sensedDataBenda;
                std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> > sensedWalls;
                std::vector<std::tuple<Object::Ptr, Eigen::Vector3d> > sensedObjects;

                // for statistics
                std::vector<Eigen::Vector3d> truePath;
                std::vector<int> finalpose;

                double nearestLandmark;

                void move();
                void turn(int direction, double angle);
                void sense();

                double _moveStep;

            private:
                Simulator* _sim;
                const double xnoise_stdev = 1.0e-2, ynoise_stdev = 1.0e-2, pnoise_stdev  = 2.0 * M_PI/180.; 
                const double wnoise_stdev = 1.0e-2;
                const double RANGE = 5.0;
        };

        std::vector<Dinding> struktur;
        std::vector<Benda> bendabenda;
        std::vector<Wall::Ptr> trueStructure;
        std::vector<Object::Ptr> trueObjects;
        Robot *robot;
        SE2 M; //general rotation

        // list of detected landmarks
        std::vector<Dinding*> dindings; // sensors data
        std::vector<Wall::Ptr> walls; // wall landmark
        std::vector<Benda*> bendas; // sensor data
        std::vector<Object::Ptr> objects; // object landmark

        double roomwidth;
        double roomlength;

        void getNextState();

        std::vector<Dinding> getStruktur () const { return struktur; };
        std::vector<Benda> getBendaBenda () const { return bendabenda; };
        std::vector<Wall::Ptr> getStructure () const { return trueStructure; };
        std::vector<Object::Ptr> getObjects () const { return trueObjects; };

        void run();
        void generateObjects (int);
        void generateRoom ();
        void calculateRMSE (MYSLAM::Graph&);

        void dataAssociationWallKnown (Graph&, Pose::Ptr&);
        void dataAssociationObjectKnown (Graph&, Pose::Ptr&);
        void dataAssociationWallUnknown (Graph&, Pose::Ptr&);
        void dataAssociationObjectUnknown (Graph&, Pose::Ptr&);

        void writeLandmarkEstimation (Graph&);

        private:
    };
}

#endif
