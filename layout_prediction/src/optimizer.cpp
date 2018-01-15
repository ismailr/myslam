#include "layout_prediction/optimizer.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/angle_measurement.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam2d/vertex_point_xy.h>  
#include <g2o/types/slam2d/vertex_se2.h>  
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include "isam/wall2d.h"
#include "isam/pose2d.h"
#include "isam/pose2d_wall2d_factor.h"

using namespace g2o;
using namespace isam;

namespace MYSLAM {
    Optimizer::Optimizer (System& system, Graph& graph)
       : _system (&system), _graph (&graph)
    {
        _incOptimizer = new SparseOptimizerIncremental;
        _incOptimizer->setVerbose (true);

        _slamInterface = new SlamInterface (_incOptimizer);
        _slamInterface->setUpdateGraphEachN (10);
        _slamInterface->setBatchSolveEachN (100);
    }

    Optimizer::Optimizer (System& system, Graph& graph, isam::Slam& slam)
       : _system (&system), _graph (&graph), _slam (&slam)
    {
        _incOptimizer = new SparseOptimizerIncremental;
        _incOptimizer->setVerbose (true);

        _slamInterface = new SlamInterface (_incOptimizer);
        _slamInterface->setUpdateGraphEachN (10);
        _slamInterface->setBatchSolveEachN (100);
    }

    Optimizer::Optimizer (Graph& graph)
       : _graph (&graph)
    {
        _incOptimizer = new SparseOptimizerIncremental;
        _incOptimizer->setVerbose (true);

        _slamInterface = new SlamInterface (_incOptimizer);
        _slamInterface->setUpdateGraphEachN (10);
        _slamInterface->setBatchSolveEachN (100);
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
        std::vector<std::tuple<int,int> >& activeEdges = _graph->_activeEdges;
        std::map<int, Pose::Ptr>& poseMap = _graph->_poseMap;
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;
        std::map<std::tuple<int, int>, Eigen::Vector2d>& poseWallMap = _graph->_poseWallMap;

        Eigen::Matrix<double, 3, 3> poseCovMatrix;
//        poseCovMatrix.setIdentity();
        poseCovMatrix <<    1e-1, 0.0, 0.0,
                            0.0, 1e-1, 0.0,
                            0.0, 0.0, 1e-1;
        Eigen::Matrix<double, 2, 2> wallCovMatrix;
//        wallCovMatrix.setIdentity();
        wallCovMatrix <<    1e-1, 0.0,
                            0.0, 1e-1;

        PoseVertex *u; 
        for (std::vector<int>::iterator it = activePoses.begin(); it != activePoses.end(); it++)
        {
            Pose* pose = poseMap[*it].get(); 
            double& x = pose->_pose[0]; 
            double& y = pose->_pose[1];
            double& p = pose->_pose[2];

            double& xmodel = pose->_poseByModel[0]; 
            double& ymodel = pose->_poseByModel[1]; 
            double& pmodel = pose->_poseByModel[2]; 

            SE2 vse2 (x,y,p); 
            SE2 vse2_model (xmodel, ymodel, pmodel);
            PoseVertex *v = new PoseVertex; 
            v->setId (pose->_id); 
            v->setEstimate (vse2);
            v->setModel (vse2_model);
            v->setFixed (pose->_id == 0 /*|| it == activePoses.begin()*/); 
            o->addVertex (v); 

            if (it != activePoses.begin())
            {
                PoseMeasurement* pm = new PoseMeasurement;
                pm->vertices()[0] = u;
                pm->vertices()[1] = v;
//                pm->setMeasurement (u->estimate().inverse() * v->estimate());
                SE2 mu = *(u->getModel());
                SE2 mv = *(v->getModel());
                pm->setMeasurement (mu.inverse() * mv);
                pm->information () = poseCovMatrix.inverse();
                o->addEdge (pm);

            }

            u = v;
        }

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
            o->addVertex (v);

//            for (std::set<int>::iterator jt = activeWalls.begin();
//                    jt != activeWalls.end(); jt++)
//            {
//                AngleMeasurement* am = new AngleMeasurement;
//                am->vertices()[0] = o->vertex (*jt);
//                am->vertices()[1] = o->vertex (*jt);
//                o->addEdge (am);
//                Eigen::Matrix<double, 1, 1> inf;
//                inf.setIdentity();
//                am->information () = inf;
//            }
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
//            rk->setDelta (sqrt(5.99));

            o->addEdge (wm);
        }

        std::ofstream mfile;
        mfile.open ("/home/ism/tmp/data.g2o", std::ios::out | std::ios::app);
        o->save(mfile);
        o->initializeOptimization();
        o->optimize(10);
        o->save(mfile);
        mfile << std::endl << std::endl;
        mfile.close();

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

        int fixPoseForNextIter = activePoses.back();
        activePoses.clear();
        activeWalls.clear();
        activeEdges.clear();
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

            double& xmodel = pose->_poseByModel[0]; 
            double& ymodel = pose->_poseByModel[1]; 
            double& pmodel = pose->_poseByModel[2]; 

            SE2 vse2 (x,y,p);
            SE2 vse2_model (xmodel, ymodel, pmodel);
            PoseVertex *v = new PoseVertex;
            v->setId (pose->_id);
            v->setEstimate (vse2);
            v->setModel (vse2_model);
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
        }

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
        posefile.open ("/home/ism/tmp/finalpose.dat", std::ios::out);
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

    void Optimizer::incrementalOptimize(int poseId, std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> > walls)
    {
        std::map<int, Pose::Ptr>& poseMap = _graph->_poseMap;
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;
        std::map<std::tuple<int, int>, Eigen::Vector2d>& poseWallMap = _graph->_poseWallMap;

        Noise noise3 = Information (100. * eye(3));
        Noise noise2 = Information (100. * eye(2));

        Pose::Ptr poseData = poseMap[poseId];
        double x = poseData->_pose[0];
        double y = poseData->_pose[1];
        double p = poseData->_pose[2];

//        Pose2d_Node *pose_node = new Pose2d_Node();
//        _slam->add_node (pose_node);
//        _poseNodes.push_back (pose_node);

//        if (poseId == 0)
//        {
//            Pose2d origin (x, y, p);
//            Pose2d_Factor *prior = new Pose2d_Factor (_poseNodes[0], origin, noise3);
//            _slam->add_factor (prior);
//        }
//
//        Pose2d_Pose2d_Factor *constraint = 
//            new Pose2d_Pose2d_Factor (_poseNodes.back(), _poseNodes.rbegin()[1], poseData->_measurement, noise3);
//        _slam->add_factor (constraint);
//
//        for (int i = 0; i < walls.size(); i++)
//        {
            Wall2d_Node *wall_node = new Wall2d_Node();
//            _slam->add_node (wall_node);
//
//            Wall2d measure (poseWallMap[std::tuple<int,int>(poseId,walls[i]->_id)]); 
//            Pose2d_Wall2d_Factor *measurement = 
//                new Pose2d_Wall2d_Factor (_poseNodes.back(), wall_node, measure, noise2);
//
//            _slam->add_factor (measurement);
//        }
    }
}
