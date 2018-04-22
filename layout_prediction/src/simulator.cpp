#include <iostream>
#include <fstream>

#include "layout_prediction/simulator.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/settings.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {

    Simulator::Robot::Robot(Simulator* sim) 
        : _sim (sim), _moveStep (0.5), nearestLandmark (100.0)
    {
        truePose.setZero(); 
        simPose.setZero();
        truePose[2] = M_PI;
        simPose[2] = truePose[2];
    }

    void Simulator::Robot::move() {

        if (nearestLandmark < 1.0) {
            double _rot = uniform_generator<double>(-M_PI, M_PI);
            int _dir100 = uniform_generator<int>(1,100);

            int _dir;
            _dir100 > 50 ? _dir = 1 : _dir = 0;

            turn (_dir, _rot);
            return;
        }

        SE2 pose;
        pose.fromVector (truePose);  

        SE2 move (_moveStep, 0.0, 0.0);
        pose = pose * move;
        truePose = pose.toVector();
//        truePath.push_back(truePose);

        SE2 simpose;
        simpose.fromVector(simPose);

        SE2 simmove (
                move.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
//                move.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
//                move.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
                0.0, 0.0);
        simpose = simpose * simmove;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::turn (int direction, double angle)
    {
        direction == 0 ? angle = angle : angle = -angle;
//        SE2 turn (_moveStep, 0.0, angle);
        SE2 turn (0.0, 0.0, angle);

        SE2 pose;
        pose.fromVector (truePose);
        pose = pose * turn;
        truePose = pose.toVector();

        SE2 simpose;
        simpose.fromVector (simPose);

        SE2 simturn (
                turn.translation().x() + gaussian_generator<double>(0.0, xnoise_stdev),
//                turn.translation().y() + gaussian_generator<double>(0.0, ynoise_stdev),
                0.0, 
//                0.0, 
                turn.rotation().angle() + gaussian_generator<double>(0.0, pnoise_stdev));
        simpose = simpose * simturn;
        simPose = simpose.toVector();
    }

    void Simulator::Robot::sense()
    {
        double _nearestLandmark = 100.0;

        sensedData.clear();
        sensedDinding.clear();
        sensedDataBenda.clear();
        sensedBenda.clear();

        SE2 simpose; simpose.fromVector (simPose);
        SE2 truepose; truepose.fromVector (truePose);

        // sensor model
        for (std::vector<Dinding>::iterator it = _sim->struktur.begin(); it != _sim->struktur.end(); it++)
        {
            double unoise = gaussian_generator<double>(0.0, wnoise_stdev);
            double vnoise = gaussian_generator<double>(0.0, wnoise_stdev);

            double u = (*it).xx;
            double v = (*it).xy;

            Eigen::Vector2d xx (u, v);
            Eigen::Vector2d xx_ = truepose.inverse() * xx;

            Eigen::Vector2d data (xx_[0] + unoise, xx_[1] + vnoise);
//
//            double r = sqrt (xx_[0]*xx_[0] + xx_[1]*xx_[1]);
//            double angle = normalize_theta (atan2 (xx_[1],xx_[0]));
//
            double m = (*it).m;
            double c = (*it).c;

            double& x = truePose[0];
            double& y = truePose[1];
            double& t = truePose[2];
            double cost = cos(t);
            double sint = sin(t);

            double m_ = (-sint+m*cost)/(cost+m*sint);
            double c_ = (c-y+m*x)/(cost+m*sint);

            double u_ = (-m_*c_)/(m_*m_+1) + unoise;
            double v_ = c_/(m_*m_+1) + vnoise;

            double r = sqrt (u_*u_ + v_*v_);
//            double r_ = std::abs (-c_/m_);            
            double angle = normalize_angle (atan2 (v_,u_));

            if (r <= RANGE)
            {
               if ((angle >= 0.0 && angle < 0.5*M_PI) || (angle > 1.5*M_PI && angle <= 2*M_PI))
                {
                    if (r< _nearestLandmark) _nearestLandmark = r;
                    sensedDinding.push_back (&(*it));
                    sensedData.push_back (data);
                }
            }
        }

        for (std::vector<Benda>::iterator it = _sim->bendabenda.begin(); it != _sim->bendabenda.end(); it++)
        {
            double oxnoise = gaussian_generator<double>(0.0, 1.0e-2);
            double oynoise = gaussian_generator<double>(0.0, 1.0e-2);
            double otnoise = gaussian_generator<double>(0.0, 2.0*M_PI/180.0);

            SE2 objectPose; objectPose.fromVector (it->pose);

            SE2 objectMeasurement = truepose.inverse() * objectPose;

            SE2 objectMeasurementWithNoise (
                   objectMeasurement.toVector()[0] + oxnoise, 
                   objectMeasurement.toVector()[1] + oynoise, 
                   objectMeasurement.toVector()[2] + otnoise); 

            double& omx = objectMeasurementWithNoise.toVector()[0];
            double& omy = objectMeasurementWithNoise.toVector()[1];
            double& omt = objectMeasurementWithNoise.toVector()[2];

            double r = sqrt (omx*omx + omy*omy);
            double angle = normalize_theta (atan2 (omy,omx));

//            if (r < _nearestLandmark) _nearestLandmark = r;

            if (r <= RANGE)
            {
                if ((angle >= 0.0 && angle < M_PI/2) || (angle > -2*M_PI && angle <= -M_PI/2))
                {
                    sensedBenda.push_back (&(*it));
                    sensedDataBenda.push_back (objectMeasurementWithNoise.toVector());
                }
            }
        }

        nearestLandmark = _nearestLandmark;
    }

    Simulator::Simulator()
    {
        double rot = uniform_generator<double>(-M_PI, M_PI);
        Eigen::Vector3d r (0.0, 0.0, rot);
        M.fromVector(r);

        roomwidth = uniform_generator<double>(0.0, 10.0) + 5.0;
        roomlength = uniform_generator<double>(0.0, 10.0 ) + 5.0;

        generateRoom();
        generateObjects (MYSLAM::SIM_NUMBER_OF_OBJECTS);

        robot = new Robot(this);
    }

    void Simulator::getNextState()
    {
        ofstream mfile;
        mfile.open ("/home/ism/tmp/sim.dat", ios::out | ios::app);
        mfile   << robot->simPose[0] << " " << robot->simPose[1] << " " << robot->simPose[2] << " " 
                << robot->truePose[0] << " " << robot->truePose[1] << " " << robot->truePose[2] << std::endl;
        mfile.close();

        robot->move();
        robot->sense();
    }

    void Simulator::run()
    {
        MYSLAM::Graph graph;
        Optimizer o(graph);

        Eigen::Vector3d lastSimPose;
        Pose::Ptr lastPose;

        // first pose
        Pose::Ptr firstPose (new Pose);
        firstPose->_pose = robot->simPose;
        graph.insertNode (firstPose);
        lastSimPose = robot->simPose;
        lastPose = firstPose;

        // stats
        robot->truePath.push_back (robot->truePose);
        robot->finalpose.push_back (firstPose->_id); // end of stats

        getNextState();

        for (int frame = 1; frame < MYSLAM::SIM_NUMBER_OF_ITERATIONS; frame++)
        {
            SE2 lastodom; lastodom.fromVector (lastSimPose);
            SE2 currentodom; currentodom.fromVector (robot->simPose);
            SE2 delta = lastodom.inverse() * currentodom;

            SE2 lastpose; lastpose.fromVector (lastPose->_pose);
            Pose::Ptr pose (new Pose);
            pose->_pose = (lastpose * delta).toVector();
            graph.insertNode (pose);

            // stats
            robot->truePath.push_back (robot->truePose);
            robot->finalpose.push_back (pose->_id); // end of stats

            dataAssociationWallKnown (graph, pose);
            dataAssociationObjectKnown (graph, pose);

            int N = graph._activePoses.size();
            int M = graph._activeWalls.size();
            int O = graph._activeObjects.size();
            int P = graph._activePoseObjectEdges.size();
            int E = graph._activeEdges.size() + N - 1 + P;
            int V = 3*N + 2*M + 3*O;

//            std::ofstream lfile;
//            lfile.open ("/home/ism/tmp/numoflandmark.dat",std::ios::out | std::ios::app);
//            lfile   << frame << " " 
//                    << N << " " << M << " " << E << " " << V << std::endl;
//            lfile.close();

//            if (frame % 5 == 0)
//            if (graph._activeWalls.size() >= 2 && graph._activePoses.size() >= 2)
//            if (E >= V)
            if ((M >= 1 && N >= 9 && O >= 2) || frame + 1 == MYSLAM::SIM_NUMBER_OF_ITERATIONS)
            {
                o.localOptimize();
            }

            lastSimPose = robot->simPose;
            lastPose = pose;

            getNextState();
        }

//        o.globalOptimize();
        writeLandmarkEstimation (graph);
//        calculateRMSE (graph);
    }

    void Simulator::generateObjects (int n) {

        ofstream objectfile;
        objectfile.open ("/home/ism/tmp/trueobject.dat", std::ios::out | std::ios::app);

        for (int i = 0; i < n; ++i) {
            double x = uniform_generator<double>(-roomwidth + 0.5, roomwidth - 0.5);
            double y = uniform_generator<double>(-roomlength + 0.5, roomlength - 0.5);
            double t = uniform_generator<double>(-M_PI,M_PI);

            Benda b;
            b.id = i;
            SE2 o (x, y, t);
            o = M * o;
            b.pose = o.toVector();
            bendabenda.push_back (b);
            objectfile << b.pose.transpose() << std::endl;
        }

        objectfile.close();
    }

    void Simulator::generateRoom () {

        std::vector<Eigen::Vector2d> data;

        Eigen::Vector2d c1 (0.0, roomlength);
        Eigen::Vector2d p1 (-roomwidth, roomlength);
        Eigen::Vector2d q1 (roomwidth, roomlength);
        data.push_back(c1);
        data.push_back(p1);
        data.push_back(q1);

        Eigen::Vector2d c2 (0.0, -roomlength);
        Eigen::Vector2d p2 (-roomwidth, -roomlength);
        Eigen::Vector2d q2 (roomwidth, -roomlength);
        data.push_back(c2);
        data.push_back(p2);
        data.push_back(q2);

        Eigen::Vector2d c3 (-roomwidth, 0.0);
        Eigen::Vector2d p3 (-roomwidth, -roomlength);
        Eigen::Vector2d q3 (-roomwidth, roomlength);
        data.push_back(c3);
        data.push_back(p3);
        data.push_back(q3);

        Eigen::Vector2d c4 (roomwidth, 0.0);
        Eigen::Vector2d p4 (roomwidth, -roomlength);
        Eigen::Vector2d q4 (roomwidth, roomlength);
        data.push_back(c4);
        data.push_back(p4);
        data.push_back(q4);

        ofstream wfile;
        wfile.open ("/home/ism/tmp/truewall.dat", std::ios::out | std::ios::app);
        for (int i = 0, j = 0; i < 4; i++, j = j + 3)
        {
            Dinding *d = new Dinding;
            d->id = i;
            data[j] = M * data[j];
            d->xx = data[j][0];
            d->xy = data[j][1];
            d->calcMC();
            d->p = M * data[j+1];
            d->q = M * data[j+2];
            struktur.push_back(*d);
            wfile << d->p.transpose() << " " << d->q.transpose() << std::endl;
        }
        wfile.close();
    }

    void Simulator::calculateRMSE (MYSLAM::Graph& graph) {
        std::ofstream rmsefile;
        rmsefile.open ("/home/ism/tmp/rmse.dat",std::ios::out|std::ios::app);

        double dtrans, drot, cumtrans = 0.0, cumrot = 0.0;
        int n = robot->finalpose.size();
        for (int i = 0; i < n; ++i) {
            int id = robot->finalpose[i];

            if (graph._poseMap[id]) {
                double x0 = robot->truePath[i][0];
                double y0 = robot->truePath[i][1];
                double t0 = robot->truePath[i][2];
                double x1 = graph._poseMap[id]->_pose[0];
                double y1 = graph._poseMap[id]->_pose[1];
                double t1 = graph._poseMap[id]->_pose[2];

                dtrans = (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1);
                drot = normalize_theta (t0-t1); 
                drot = drot*drot;

                cumtrans += dtrans;
                cumrot += drot;
            }
        }

        double rmsetrans = sqrt (cumtrans/n);
        double rmserot = sqrt (cumrot/n);

        rmsefile << rmsetrans << " " << normalize_angle (rmserot) << std::endl;
        rmsefile.close();
    }

    void Simulator::Dinding::calcMC() {
        m = -xx/xy;
        c = (xx*xx + xy*xy)/xy;
    }

    Simulator::Benda::Benda() {
       classid = uniform_generator<int> (1,MYSLAM::SIM_NUMBER_OF_OBJECT_CLASS);
    }

    void Simulator::dataAssociationWallKnown (Graph& graph, Pose::Ptr& pose) {

        for (size_t i = 0; i < robot->sensedDinding.size(); i++)
        {
            // asosiasi data
            Wall::Ptr w = NULL;
            Dinding* d = robot->sensedDinding[i];
            bool newLandmark = true;

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
                        newLandmark = false;
                        break;
                    }
                    else if (j == dindings.size() - 1)
                    {
                        Wall::Ptr wall (new Wall);
                        w = wall;
                        w->_line.p = d->p;
                        w->_line.q = d->q;
                        w->initParams();
                        dindings.push_back (d);
                        walls.push_back (w);
                        break;
                    }
                }
            }

            if (newLandmark) {
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
            }

            graph.insertNode (w);
            std::tuple<int, int> e (pose->_id, w->_id);
            graph.insertPoseWallEdge (e, robot->sensedData[i]);
        }

    }

    void Simulator::dataAssociationObjectKnown (Graph& graph, Pose::Ptr& pose) {

        for (size_t i = 0; i < robot->sensedBenda.size(); i++)
        {
            // asosiasi data
            Object::Ptr o = NULL;
            Benda* b = robot->sensedBenda[i];
            bool newObject = true;

            if (bendas.size() == 0)
            {
                Object::Ptr object (new Object);
                o = object;
                bendas.push_back (b);
                objects.push_back (o);

            } else {
                for (int j = 0; j < bendas.size(); j++)
                {
                    if (b->id == bendas[j]->id)
                    {
                        o = objects[j];
                        newObject = false;
                        break;
                    }
                    else if (j == bendas.size() - 1)
                    {
                        Object::Ptr object (new Object);
                        o = object;
                        bendas.push_back (b);
                        objects.push_back (o);
                        break;
                    }
                }
            }

            if (newObject) {
                Eigen::Vector3d m = robot->sensedDataBenda[i];
                SE2 measurement; measurement.fromVector (m); 
                SE2 robotPose; robotPose.fromVector (pose->_pose);
                SE2 objectPose = robotPose * measurement;
                o->_pose = objectPose.toVector();
            }

            graph.insertNode (o);
            std::tuple<int, int> e (pose->_id, o->_id);
            graph.insertPoseObjectEdge (e, robot->sensedDataBenda[i]);
        }
    }

    void Simulator::dataAssociationWallUnknown (Graph& graph, Pose::Ptr& pose) {

        for (size_t i = 0; i < robot->sensedDinding.size(); i++)
        {

        }

    }

    void Simulator::dataAssociationObjectUnknown (Graph& graph, Pose::Ptr& pose) {

        for (size_t i = 0; i < robot->sensedBenda.size(); i++)
        {

        }

    }

    void Simulator::writeLandmarkEstimation (Graph& graph) {

        ofstream wfile;
        wfile.open ("/home/ism/tmp/wall.dat", std::ios::out | std::ios::app);
        for (std::map<int, Wall::Ptr>::iterator it = graph._wallMap.begin();
                it != graph._wallMap.end(); it++)
        {
            wfile << it->second->_line.p.transpose() << " " << it->second->_line.q.transpose() << std::endl;
        }
        wfile.close();

        ofstream ofile;
        ofile.open ("/home/ism/tmp/object.dat", std::ios::out | std::ios::app);
        for (std::map<int, Object::Ptr>::iterator it = graph._objectMap.begin();
                it != graph._objectMap.end(); it++)
        {
            ofile << it->second->_pose.transpose() << std::endl;
        }
        ofile.close();
    }
}
