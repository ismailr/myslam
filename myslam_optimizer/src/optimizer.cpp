#include "layout_prediction/optimizer.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/object.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/point.h"
#include "layout_prediction/angle_measurement.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/wall_measurement.h"
#include "layout_prediction/object_measurement.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

using namespace g2o;

namespace MYSLAM {
    Optimizer::Optimizer (System& system, Graph& graph)
       : _system (&system), _graph (&graph)
    {
    }

    Optimizer::Optimizer (Graph& graph)
       : _graph (&graph)
    {
    }

    void Optimizer::localOptimize()
    {
        typedef BlockSolver< BlockSolverTraits<-1,-1> > SlamBlockSolver;
        typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

        g2o::SparseOptimizer *o = new g2o::SparseOptimizer;
        auto linearSolver = g2o::make_unique<SlamLinearSolver>();
        linearSolver->setBlockOrdering (false);
        OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg (
                g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
        o->setAlgorithm (solver);
        o->setVerbose (false);

        std::vector<int>& activePoses = _graph->_activePoses;
        std::set<int>& activeWalls = _graph->_activeWalls;
        std::set<int>& activeObjects = _graph->_activeObjects;
        std::set<int>& lastActiveWalls = _graph->_lastActiveWalls;
        std::set<int>& lastActiveObjects = _graph->_lastActiveObjects;
        std::vector<std::tuple<int,int> >& activeEdges = _graph->_activeEdges;
        std::vector<std::tuple<int,int> >& activePoseObjectEdges = _graph->_activePoseObjectEdges;
        std::map<int, Pose::Ptr>& poseMap = _graph->_poseMap;
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;
        std::map<int, Object::Ptr>& objectMap = _graph->_objectMap;
        std::map<std::tuple<int, int>, Eigen::Vector2d>& poseWallMap = _graph->_poseWallMap;
        std::map<std::tuple<int, int>, Eigen::Vector3d>& poseObjectMap = _graph->_poseObjectMap;

        Eigen::Matrix<double, 3, 3> poseCovMatrix;
//        poseCovMatrix.setIdentity();
        poseCovMatrix <<    1.0e-4, 0.0, 0.0,
                            0.0, 1.0e-4, 0.0,
                            0.0, 0.0, 4.*M_PI/180.*M_PI/180.;
        Eigen::Matrix<double, 2, 2> wallCovMatrix;
//        wallCovMatrix.setIdentity();
        wallCovMatrix <<    1.0e-4, 0.0,
                            0.0, 1.0e-4;
        Eigen::Matrix<double, 3, 3> objectCovMatrix;
//        poseCovMatrix.setIdentity();
        objectCovMatrix <<    1.0e-4, 0.0, 0.0,
                            0.0, 1.0e-4, 0.0,
                            0.0, 0.0, 4.*M_PI/180.*M_PI/180.;

        PoseVertex *u; 
        for (std::vector<int>::iterator it = activePoses.begin(); it != activePoses.end(); it++)
        {
            Pose* pose = poseMap[*it].get(); 
            double& x = pose->_pose[0]; 
            double& y = pose->_pose[1];
            double& p = pose->_pose[2];

            SE2 vse2 (x,y,p); 
            PoseVertex *v = new PoseVertex; 
            v->setId (pose->_id); 
            v->setEstimate (vse2);
            v->setFixed (pose->_id == 0 || it == activePoses.begin()); 
            o->addVertex (v); 

            if (it != activePoses.begin())
            {
                PoseMeasurement* pm = new PoseMeasurement;
                pm->vertices()[0] = u;
                pm->vertices()[1] = v;
                pm->setMeasurement (u->estimate().inverse() * v->estimate());
                pm->information () = poseCovMatrix.inverse();

//                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//                pm->setRobustKernel (rk);
//                rk->setDelta (sqrt(5.99));

                o->addEdge (pm);
            }

            u = v;
        }

//        // Adding last wall to current optimization
//        bool lastActiveWallsEmpty = _graph->_lastActiveWalls.empty();

        for (std::set<int>::iterator it = activeWalls.begin();
                it != activeWalls.end(); it++)
        {
            Wall* wall= wallMap[*it].get();
            double& xx = wall->_line.xx[0];
            double& xy = wall->_line.xx[1];

            double data[2] = {xx,xy};
            WallVertex *v = new WallVertex;
            v->setId (wall->_id);
            v->setEstimateDataImpl (data);
//            v->setMarginalized (true);
            o->addVertex (v);
        }

        for (std::set<int>::iterator it = activeObjects.begin();
                it != activeObjects.end(); it++)
        {
            Object* object = objectMap[*it].get(); 
            double& x = object->_pose[0]; 
            double& y = object->_pose[1];
            double& p = object->_pose[2];

            SE2 vse2 (x,y,p); 
            ObjectVertex *v = new ObjectVertex; 
            v->setId (object->_id); 
            v->setEstimate (vse2);
//            v->setMarginalized (true);
            o->addVertex (v); 
        }

        for (std::vector<std::tuple<int,int> >::iterator it = activeEdges.begin();
                it != activeEdges.end(); it++)
        {
            double& xx_ = poseWallMap[*it].x();
            double& xy_ = poseWallMap[*it].y();
            double data[2] = {xx_,xy_};

            WallMeasurement* wm = new WallMeasurement;

            wm->vertices()[0] = o->vertex (std::get<0>(*it));
            wm->vertices()[1] = o->vertex (std::get<1>(*it));
            wm->setMeasurementData (data);
            wm->information() = wallCovMatrix.inverse();
            
//            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//            wm->setRobustKernel (rk);
//            rk->setDelta (sqrt(1000.99));

            o->addEdge (wm);
        }

        for (std::vector<std::tuple<int,int> >::iterator it = activePoseObjectEdges.begin();
                it != activePoseObjectEdges.end(); it++)
        {
            SE2 data; data.fromVector (poseObjectMap[*it]);

            ObjectMeasurement* om = new ObjectMeasurement;

            om->vertices()[0] = o->vertex (std::get<0>(*it));
            om->vertices()[1] = o->vertex (std::get<1>(*it));
            om->setMeasurement (data);
            om->information() = objectCovMatrix.inverse();
            
//            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//            om->setRobustKernel (rk);
//            rk->setDelta (sqrt(1000.99));

            o->addEdge (om);
        }

//        std::ofstream mfile;
//        mfile.open ("/home/ism/tmp/data.g2o", std::ios::out | std::ios::app);
//        o->save(mfile);
        o->initializeOptimization();
        o->optimize(10);
//        o->save(mfile);
//        mfile << std::endl << std::endl;
//        mfile.close();

        std::ofstream posefile;
        posefile.open ("/home/ism/tmp/finalpose.dat", std::ios::out | std::ios::app);
        for (std::vector<int>::iterator it = activePoses.begin();
                it != activePoses.end(); it++)
        {
            PoseVertex* optv = dynamic_cast<PoseVertex*> (o->vertex (*it));
            double x = optv->estimate().translation().x();
            double y = optv->estimate().translation().y();
            double p = optv->estimate().rotation().angle();

            poseMap[*it]->_pose[0] = x;
            poseMap[*it]->_pose[1] = y;
            poseMap[*it]->_pose[2] = p;

            if (*it == 0 || it != activePoses.begin())
                posefile << x << " " << y << " " << p << std::endl;
        }
        posefile.close();

        for (std::set<int>::iterator it = activeWalls.begin();
                it != activeWalls.end(); it++)
        {
            WallVertex* optv = dynamic_cast<WallVertex*> (o->vertex (*it));
            double xx = optv->estimate().x();
            double xy = optv->estimate().y();

            wallMap[*it]->_line.xx[0] = xx;
            wallMap[*it]->_line.xx[1] = xy;
            wallMap[*it]->updateParams ();
        }

        for (std::set<int>::iterator it = activeObjects.begin();
                it != activeObjects.end(); it++)
        {
            ObjectVertex* optv = dynamic_cast<ObjectVertex*> (o->vertex (*it));
            double x = optv->estimate().translation().x();
            double y = optv->estimate().translation().y();
            double p = optv->estimate().rotation().angle();

            objectMap[*it]->_pose[0] = x;
            objectMap[*it]->_pose[1] = y;
            objectMap[*it]->_pose[2] = p;
        }

        int fixPoseForNextIter = activePoses.back();
        activePoses.clear();
        lastActiveWalls.clear();
        lastActiveWalls = activeWalls;
        lastActiveObjects.clear();
        lastActiveObjects = activeObjects;
        activeWalls.clear();
        activeObjects.clear();
        activeEdges.clear();
        activePoseObjectEdges.clear();
        activePoses.push_back (fixPoseForNextIter);
    }

    void Optimizer::globalOptimize()
    {
        typedef BlockSolver< BlockSolverTraits<-1,-1> > SlamBlockSolver;
        typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

        g2o::SparseOptimizer *o = new g2o::SparseOptimizer;
        auto linearSolver = g2o::make_unique<SlamLinearSolver>();
        linearSolver->setBlockOrdering (false);
        OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg (
                g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
        o->setAlgorithm (solver);
        o->setVerbose (true);

        std::map<int, Pose::Ptr>& poseMap = _graph->_poseMap;
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;
        std::map<std::tuple<int, int>, Eigen::Vector2d>& poseWallMap = _graph->_poseWallMap;

        Eigen::Matrix<double, 3, 3> poseInfMatrix;
        poseInfMatrix.setIdentity();
        Eigen::Matrix<double, 2, 2> wallInfMatrix;
        wallInfMatrix.setIdentity();

        PoseVertex *u; 
        for (std::map<int, Pose::Ptr>::iterator it = poseMap.begin(); it != poseMap.end(); it++)
        {
            Pose* pose = it->second.get();
            double& x = pose->_pose[0];
            double& y = pose->_pose[1];
            double& p = pose->_pose[2];

            SE2 vse2 (x,y,p);
            PoseVertex *v = new PoseVertex;
            v->setId (pose->_id);
            v->setEstimate (vse2);
            v->setFixed (pose->_id == 0);
            o->addVertex (v);

            if (it != poseMap.begin())
            {
                PoseMeasurement* pm = new PoseMeasurement;
                pm->vertices()[0] = u;
                pm->vertices()[1] = v;
                pm->setMeasurement (u->estimate().inverse() * v->estimate());
                pm->information () = poseInfMatrix;
                o->addEdge (pm);
            }

            u = v;

//            // hack
//            if (std::next(it) == poseMap.end())
//            {
//                PoseMeasurement* pm = new PoseMeasurement;
//                pm->vertices()[0] = v;
//                pm->vertices()[1] = o->vertex(0);
//                SE2 zero;
//                pm->setMeasurement (zero);
//                pm->information () = poseInfMatrix;
//                o->addEdge (pm);
//            }
        }

//        WallVertex* t;
        for (std::map<int, Wall::Ptr>::iterator it = wallMap.begin();
                it != wallMap.end(); it++)
        {
            Wall* wall = it->second.get(); 
            double& xx = wall->_line.xx[0];
            double& xy = wall->_line.xx[1];

            double data[2] = {xx,xy};
            WallVertex *v = new WallVertex;
            v->setId (wall->_id);
            v->setEstimateDataImpl (data);
            v->setMarginalized (true);
            o->addVertex (v);
        }

        for (std::map<std::tuple<int,int>, Eigen::Vector2d>::iterator it = poseWallMap.begin();
                it != poseWallMap.end(); it++)
        {
            double& xx_ = it->second.x();
            double& xy_ = it->second.y();
            double data[2] = {xx_,xy_};

            WallMeasurement* wm = new WallMeasurement;

            wm->vertices()[0] = o->vertex (std::get<0>(it->first));
            wm->vertices()[1] = o->vertex (std::get<1>(it->first));
            wm->setMeasurementData (data);
            wm->information() = wallInfMatrix;
            o->addEdge (wm);
        }

        o->initializeOptimization();
        o->optimize(100);

        std::ofstream posefile;
        posefile.open ("/home/ism/tmp/finalpose.dat", std::ios::out | std::ios::app);
        for (std::map<int,Pose::Ptr>::iterator it = poseMap.begin();
                it != poseMap.end(); it++)
        {
            int id = it->first;
            PoseVertex* optv = dynamic_cast<PoseVertex*> (o->vertex (id));
            double x = optv->estimate().translation().x();
            double y = optv->estimate().translation().y();
            double p = optv->estimate().rotation().angle();

            poseMap[id]->_pose[0] = x;
            poseMap[id]->_pose[1] = y;
            poseMap[id]->_pose[2] = p;
            posefile << x << " " << y << " " << p << std::endl;
        }
        posefile.close();

        for (std::map<int,Wall::Ptr>::iterator it = wallMap.begin();
                it != wallMap.end(); it++)
        {
            int id = it->first;
            WallVertex* optv = dynamic_cast<WallVertex*> (o->vertex (id));
            double xx = optv->estimate().x();
            double xy = optv->estimate().y();

            wallMap[id]->_line.xx[0] = xx;
            wallMap[id]->_line.xx[1] = xy;
            wallMap[id]->updateParams ();
        }
    }

    void Optimizer::localOptimize2(Pose::Ptr pose)
    {
        if (pose->_detectedObjects.empty()) return;

        typedef BlockSolver< BlockSolverTraits<-1,-1> > SlamBlockSolver;
        typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

        g2o::SparseOptimizer *o = new g2o::SparseOptimizer;
        auto linearSolver = g2o::make_unique<SlamLinearSolver>();
        linearSolver->setBlockOrdering (false);
        OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg (
                g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
        o->setAlgorithm (solver);
        o->setVerbose (true);

        std::map<int, Pose::Ptr>& poseMap = _graph->_poseMap;
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;
        std::map<int, Object::Ptr>& objectMap = _graph->_objectMap;
        std::map<std::tuple<int, int>, Eigen::Vector2d>& poseWallMap = _graph->_poseWallMap;
        std::map<std::tuple<int, int>, Eigen::Vector3d>& poseObjectMap = _graph->_poseObjectMap;
        std::vector<int>& path = _graph->_path;

        Eigen::Matrix<double, 3, 3> poseCovMatrix;
//        poseCovMatrix.setIdentity();
        poseCovMatrix <<    1.0e-4, 0.0, 0.0,
                            0.0, 1.0e-4, 0.0,
                            0.0, 0.0, 4.*M_PI/180.*M_PI/180.;
        Eigen::Matrix<double, 2, 2> wallCovMatrix;
//        wallCovMatrix.setIdentity();
        wallCovMatrix <<    1.0e-4, 0.0,
                            0.0, 1.0e-4;
        Eigen::Matrix<double, 3, 3> objectCovMatrix;
//        poseCovMatrix.setIdentity();
        objectCovMatrix <<  1.0e-4, 0.0, 0.0,
                            0.0, 1.0e-4, 0.0,
                            0.0, 0.0, 4.*M_PI/180.*M_PI/180.;

        std::set<int> activeObjects = pose->_detectedObjects;

        std::vector<int> activePoses;
        for (auto it = path.rbegin(); it != path.rend(); it++) {
            std::set<int> obj = poseMap[*it]->_detectedObjects;

            for (auto jt = obj.begin(); jt != obj.end(); jt++) {
                auto kt = std::find (activeObjects.begin(), activeObjects.end(), *jt);

                if (kt != activeObjects.end()) {
                    auto lt = std::find (activePoses.begin(), activePoses.end(), *it);

                    if (lt == activePoses.end())
                        activePoses.push_back(*it);
                }
            }
        }

        std::reverse (activePoses.begin(), activePoses.end());

        if (activePoses.size() == 1) return;

        PoseVertex *u; 
        for (auto it = activePoses.begin(); it != activePoses.end(); it++)
        {
            Pose* p = poseMap[*it].get(); 
            double& x = pose->_pose[0]; 
            double& y = pose->_pose[1];
            double& t = pose->_pose[2];

            SE2 vse2 (x,y,t); 
            PoseVertex *v = new PoseVertex; 
            v->setId (p->_id); 
            v->setEstimate (vse2);
            v->setFixed (it == activePoses.begin());
            o->addVertex (v); 

            if (it != activePoses.begin())
            {
                PoseMeasurement* pm = new PoseMeasurement;
                pm->vertices()[0] = u;
                pm->vertices()[1] = v;
                pm->setMeasurement (u->estimate().inverse() * v->estimate());
                pm->information () = poseCovMatrix.inverse();

//                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//                pm->setRobustKernel (rk);
//                rk->setDelta (sqrt(5.99));

                o->addEdge (pm);
            }

            u = v;
        }

        for (auto it = activeObjects.begin(); it != activeObjects.end(); it++)
        {
            Object* object = objectMap[*it].get(); 
            double& x = object->_pose[0]; 
            double& y = object->_pose[1];
            double& p = object->_pose[2];

            SE2 vse2 (x,y,p); 
            ObjectVertex *v = new ObjectVertex; 
            v->setId (object->_id); 
            v->setEstimate (vse2);
            v->setMarginalized (true);
            o->addVertex (v); 

            for (auto jt = activePoses.begin(); jt != activePoses.end(); jt++) {

                if (poseObjectMap.count(std::make_tuple(*jt,*it)) == 0) continue;

                ObjectMeasurement* om = new ObjectMeasurement;

                om->vertices()[0] = o->vertex (*jt);
                om->vertices()[1] = o->vertex (*it);
                om->setMeasurement (poseObjectMap[std::make_tuple(*jt, *it)]);
                om->information() = objectCovMatrix.inverse();
                
    //            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    //            om->setRobustKernel (rk);
    //            rk->setDelta (sqrt(1000.99));

                o->addEdge (om);
            }
        }

//        for (auto it = wallMap.begin(); it != wallMap.end(); it++)
//        {
//            Wall* wall= (it->second).get();
//            double& xx = wall->_line.xx[0];
//            double& xy = wall->_line.xx[1];
//
//            double data[2] = {xx,xy};
//            WallVertex *v = new WallVertex;
//            v->setId (wall->_id);
//            v->setEstimateDataImpl (data);
////            v->setMarginalized (!wall->_active);
//            v->setMarginalized (true);
//            o->addVertex (v);
//        }
//
//        for (auto it = poseWallMap.begin(); it != poseWallMap.end(); it++)
//        {
//            double& xx_ = (it->second).x();
//            double& xy_ = (it->second).y();
//            double data[2] = {xx_,xy_};
//
//            WallMeasurement* wm = new WallMeasurement;
//
//            wm->vertices()[0] = o->vertex (std::get<0>(it->first));
//            wm->vertices()[1] = o->vertex (std::get<1>(it->first));
//            wm->setMeasurementData (data);
//            wm->information() = wallCovMatrix.inverse();
//            
////            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
////            wm->setRobustKernel (rk);
////            rk->setDelta (sqrt(5.99));
//
//            o->addEdge (wm);
//        }

        o->initializeOptimization(); 
        o->optimize(10);

        std::cout << "RECOVER POSE" << std::endl;
        std::ofstream posefile;
        posefile.open ("/home/ism/tmp/finalpose.dat", std::ios::out | std::ios::app);
        for (auto it = activePoses.begin(); it != activePoses.end(); it++)
        {
            PoseVertex* optv = dynamic_cast<PoseVertex*> (o->vertex (*it));
            double x = optv->estimate().translation().x();
            double y = optv->estimate().translation().y();
            double p = optv->estimate().rotation().angle();

            poseMap[*it]->_pose[0] = x;
            poseMap[*it]->_pose[1] = y;
            poseMap[*it]->_pose[2] = p;

            poseMap[*it]->_active = false;

            posefile << x << " " << y << " " << p << std::endl;
        }
        posefile.close();
        std::cout << "DONE" << std::endl;

//        std::cout << "RECOVER WALL" << std::endl;
//        for (auto it = wallMap.begin(); it != wallMap.end(); it++)
//        {
//            WallVertex* optv = dynamic_cast<WallVertex*> (o->vertex (it->first));
//            double xx = optv->estimate().x();
//            double xy = optv->estimate().y();
//
//            (it->second)->_line.xx[0] = xx;
//            (it->second)->_line.xx[1] = xy;
//            (it->second)->updateParams ();
//
//            (it->second)->_active = false;
//        }
//        std::cout << "DONE" << std::endl;

        std::cout << "RECOVER OBJECTS" << std::endl;
        for (auto it = activeObjects.begin(); it != activeObjects.end(); it++)
        {
            ObjectVertex* optv = dynamic_cast<ObjectVertex*> (o->vertex (*it));
            double x = optv->estimate().translation().x();
            double y = optv->estimate().translation().y();
            double p = optv->estimate().rotation().angle();

            objectMap[*it]->_pose[0] = x;
            objectMap[*it]->_pose[1] = y;
            objectMap[*it]->_pose[2] = p;

            objectMap[*it]->_active = false;
        }
        std::cout << "DONE" << std::endl;
    }
}