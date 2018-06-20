#include <iostream>
#include <fstream>
#include <tuple>
#include <chrono>
#include <thread>

#include "layout_prediction/simulator.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/settings.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/data_association.h"

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

        sensedWalls.clear();
        sensedObjects.clear();

        SE2 simpose; simpose.fromVector (simPose);
        SE2 truepose; truepose.fromVector (truePose);

        // sensor model
        for (std::vector<Wall::Ptr>::iterator it = _sim->structure.begin(); it != _sim->structure.end(); it++)
        {
            double unoise = gaussian_generator<double>(0.0, wnoise_stdev);
            double vnoise = gaussian_generator<double>(0.0, wnoise_stdev);

            double u = (*it)->_line.xx[0];
            double v = (*it)->_line.xx[1];

            Eigen::Vector2d xx (u, v);
            Eigen::Vector2d xx_ = truepose.inverse() * xx;

            Eigen::Vector2d data (xx_[0] + unoise, xx_[1] + vnoise);

            double m = (*it)->_line.mc[0];
            double c = (*it)->_line.mc[1];

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
            double angle = normalize_angle (atan2 (v_,u_));

            if (r <= RANGE)
            {
               if ((angle >= 0.0 && angle < 0.5*M_PI) || (angle > 1.5*M_PI && angle <= 2*M_PI))
                {
                    if (r < _nearestLandmark) _nearestLandmark = r;
                    sensedWalls.push_back(std::make_tuple(*it,data));

                    double uu = data[0]*cost - data[1]*sint + x;
                    double vv = data[0]*sint + data[1]*cost + y;

                    std::ofstream dafile;
                    dafile.open ("/home/ism/tmp/da.dat", std::ios::out | std::ios::app);
                    dafile << (*it)->_id << ": " << uu << " " << vv << std::endl;
                    dafile.close();
                }
            }
        }

        for (std::vector<Object::Ptr>::iterator it = _sim->objects.begin(); it != _sim->objects.end(); it++)
        {
            double oxnoise = gaussian_generator<double>(0.0, 1.0e-2);
            double oynoise = gaussian_generator<double>(0.0, 1.0e-2);
            double otnoise = gaussian_generator<double>(0.0, 2.0*M_PI/180.0);

            SE2 objectPose; objectPose.fromVector ((*it)->_pose);

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
                    sensedObjects.push_back (std::make_tuple (*it, objectMeasurementWithNoise.toVector()));

                    Eigen::Vector3d objectPose = (truepose * objectMeasurementWithNoise).toVector();
                    std::ofstream dafile;
                    dafile.open ("/home/ism/tmp/da.dat", std::ios::out | std::ios::app);
                    dafile << (*it)->_id << ": " << (*it)->_classid << ": " << objectPose.transpose() << std::endl;
                    dafile.close();
                }
            }
        }

        nearestLandmark = _nearestLandmark;
    }

    Simulator::Simulator() : counter (0), trueGuess (0), newObj (0)
    {
        double rot = uniform_generator<double>(-M_PI, M_PI);
        Eigen::Vector3d r (0.0, 0.0, rot);
        M.fromVector(r);

        roomwidth = uniform_generator<double>(0.0, 10.0) + 5.0;
        roomlength = uniform_generator<double>(0.0, 10.0 ) + 5.0;

        generateWalls();
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
            ofstream dafile;
            dafile.open ("/home/ism/tmp/da.dat", std::ios::out | std::ios::app);
            dafile << "========== " << frame << " ==========" << std::endl; 
            dafile.close();

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

            if (MYSLAM::DEBUG)
                std::cout << "*************** FRAME KE-" << frame << " ********************" << std::endl;

            dataAssociationWallKnown (graph, pose);
//            dataAssociationObjectKnown (graph, pose);
            double start = clock();
            dataAssociationObjectUnknown (graph, pose);

            if (MYSLAM::DEBUG)
                std::cout << "TIME: " << (double) 1000 * (clock() - start)/CLOCKS_PER_SEC << " ms" << std::endl;

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
//            using namespace std::this_thread;
//            using namespace std::chrono_literals;
//            using std::chrono::system_clock;
//
//            std::cout << "SLEEPP....." << std::endl;
//            sleep_for (2s);
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

            Object::Ptr b (new Object);
            b->_classid = uniform_generator<int>(0, MYSLAM::SIM_NUMBER_OF_OBJECT_CLASS);
            SE2 o (x, y, t);
            o = M * o;
            b->_pose = o.toVector();
            objects.push_back(b);
            objectfile << b->_pose.transpose() << std::endl;
        }

        objectfile.close();
    }

    void Simulator::generateWalls () {

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
            Wall::Ptr w (new Wall);
            data[j] = M * data[j];
            w->_line.xx[0] = data[j][0];
            w->_line.xx[1] = data[j][1];
            w->_line.p = M * data[j+1];
            w->_line.q = M * data[j+2];
            w->updateParams();
            structure.push_back(w);
            wfile << w->_line.p.transpose() << " " << w->_line.q.transpose() << std::endl;
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

    void Simulator::dataAssociationWallKnown (Graph& graph, Pose::Ptr& pose) {

        for (int i = 0; i < robot->sensedWalls.size(); i++)
        {
            // asosiasi data
            Wall::Ptr w = std::make_shared<Wall>(*(std::get<0>(robot->sensedWalls[i])));
            Eigen::Vector2d m = std::get<1>(robot->sensedWalls[i]);

            bool newLandmark = true;

            for (std::map<int, Wall::Ptr>::iterator it = graph._wallMap.begin();
                    it != graph._wallMap.end(); ++it)
            {
                if (w->_id == it->first)
                {
                    w = it->second;
                    newLandmark = false;
                    break;
                }
            }

            if (newLandmark) {
                double& xx_ = m[0];
                double& xy_ = m[1];

                double x = pose->_pose[0];
                double y = pose->_pose[1];
                double t = pose->_pose[2];
                double cost = cos(t);
                double sint = sin(t);

                w->_line.xx[0] = xx_*cost - xy_*sint + x;
                w->_line.xx[1] = xx_*sint + xy_*cost + y;
                w->updateParams();
            }

            /* we insert also old node here to make it an active node */
            graph.insertNode (w); 

            std::tuple<int, int> e (pose->_id, w->_id);
            graph.insertPoseWallEdge (e, m);
        }

    }

    void Simulator::dataAssociationObjectKnown (Graph& graph, Pose::Ptr& pose) {

        for (int i = 0; i < robot->sensedObjects.size(); i++)
        {
            // asosiasi data
            Object::Ptr o = std::make_shared<Object>(*(std::get<0>(robot->sensedObjects[i])));
            Eigen::Vector3d m = std::get<1>(robot->sensedObjects[i]);

            bool newObject = true;

            for (std::map<int, Object::Ptr>::iterator it = graph._objectMap.begin();
                    it != graph._objectMap.end(); ++it)
            {
                if (o->_id == it->first)
                {
                    o = it->second;
                    newObject = false;
                    break;
                }
            }

            if (newObject) {
                SE2 robotPose; robotPose.fromVector (pose->_pose);
                SE2 measurement; measurement.fromVector (m); 
                SE2 objectPose = robotPose * measurement;
                o->_pose = objectPose.toVector();
            }

            graph.insertNode (o);

            std::tuple<int, int> e (pose->_id, o->_id);
            graph.insertPoseObjectEdge (e, m);
        }
    }

    void Simulator::dataAssociationWallUnknown (Graph& graph, Pose::Ptr& pose) {

    }

    void Simulator::dataAssociationObjectUnknown (Graph& graph, Pose::Ptr& pose) {

        if (robot->sensedObjects.size() <= 2) return;

        if (MYSLAM::DEBUG) {
            std::cout << "CLASSIDS: "; 
            for (int i = 0; i < robot->sensedObjects.size(); i++) 
                std::cout << (std::get<0>(robot->sensedObjects[i]))->_classid << " ";
            std::cout << std::endl;

            std::cout << "GROUNDTRUTH: ";
            for (int i = 0; i < robot->sensedObjects.size(); i++) 
                std::cout << (std::get<0>(robot->sensedObjects[i]))->_id << " "; 
            std::cout << std::endl;
        }
        
        std::vector<std::tuple<int, Eigen::Vector3d> > data;

        for (int i = 0; i < robot->sensedObjects.size(); i++) {

            int classid = (std::get<0>(robot->sensedObjects[i]))->_classid;
            Eigen::Vector3d measurement = std::get<1>(robot->sensedObjects[i]);
            data.push_back (std::make_tuple (classid, measurement));
        }

        DataAssociation da (graph);
        std::vector<int> result = da.associate (pose, data);

        for (int i = 0; i < result.size(); i++) {
            Object::Ptr o;
            if (result[i] == -1) {

                if (MYSLAM::DEBUG) {
                    int clid = std::get<0>(data[i]);
                    if (graph._objectClassMap[clid].empty())
                        trueGuess++;
                    else {
                        auto it = std::find (   graph._objectClassMap[clid].begin(),
                                                graph._objectClassMap[clid].end(),
                                                std::get<0>(robot->sensedObjects[i])->_id);
                        if (it == graph._objectClassMap[clid].end()) trueGuess++;
                        else std::cout << "ALERTTTTTT!!!!!" << std::endl;
                    }
                }

                // BOOKMARK
//                o = std::get<0>(robot->sensedObjects[i]);
                Object::Ptr ob (new Object);
                o = ob;
                o->_classid = std::get<0>(data[i]);

                SE2 p; p.fromVector (pose->_pose);
                SE2 m; m.fromVector (std::get<1>(robot->sensedObjects[i]));
                o->_pose = (p*m).toVector();
                o->_seenBy.insert (pose->_id);
                pose->_detectedObjects.insert (o->_id);
                graph.insertNode (o);
            } else {
                if (MYSLAM::DEBUG) {
                    if (result[i] == std::get<0>(robot->sensedObjects[i])->_id) trueGuess++;
                    else std::cout << "ALERTTTTTT!!!!!" << std::endl;
                }
                o = graph._objectMap[result[i]];
                o->_active = true;
                o->_seenBy.insert (pose->_id);
                pose->_detectedObjects.insert (o->_id);

            }

            if (MYSLAM::DEBUG) counter++;

            std::tuple<int, int> e (pose->_id, o->_id);
            graph.insertPoseObjectEdge (e, std::get<1>(robot->sensedObjects[i]));
        }

        if (MYSLAM::DEBUG) {
            std::cout << "CANDIDATES: ";
            for (int i = 0; i < result.size(); i++) {
                if (result[i] == -1) std::cout << "+ ";
                else std::cout << result[i] << " ";
            }
            std::cout << std::endl;

            std::cout << "ACCURACY = " << (double)(trueGuess * 100)/counter << "%" << std::endl;
        }
    }

    void Simulator::writeLandmarkEstimation (Graph& graph) {

        ofstream wfile;
        wfile.open ("/home/ism/tmp/wall.dat", std::ios::out);
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
