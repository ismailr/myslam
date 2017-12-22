#include "layout_prediction/optimizer.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"

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

using namespace g2o;

namespace MYSLAM {
    Optimizer::Optimizer (System& system, Graph& graph)
       : _system (&system), _graph (&graph)
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
        o->setVerbose (true);

        std::vector<int>& activePoses = _graph->_activePoses;
        std::vector<int>& activeWalls = _graph->_activeWalls;
        std::vector<std::tuple<int,int> >& activeEdges = _graph->_activeEdges;
        std::map<int, Pose::Ptr>& poseMap = _graph->_poseMap;
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;
        std::map<std::tuple<int, int>, Eigen::Vector2d>& poseWallMap = _graph->_poseWallMap;

        Eigen::Matrix<double, 3, 3> poseInfMatrix;
        poseInfMatrix.setIdentity();
        Eigen::Matrix<double, 2, 2> wallInfMatrix;
        wallInfMatrix.setIdentity();

        PoseVertex *u; // holding PoseVertex created in it - 1 iteration
        for (std::vector<int>::iterator it = activePoses.begin();
                it != activePoses.end(); it++)
        {
            Pose* pose = poseMap[*it].get();
            double& x = pose->_pose[0];
            double& y = pose->_pose[1];
            double& p = pose->_pose[2];

            SE2 vse2 (x,y,p);
            PoseVertex *v = new PoseVertex;
            v->setId (pose->_id);
            v->setEstimate (vse2);
            v->setFixed (pose->_id == 0);
            o->addVertex (v);

            if (it != activePoses.begin())
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

        for (std::vector<int>::iterator it = activeWalls.begin();
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
            wm->information() = wallInfMatrix;
            o->addEdge (wm);
        }

        o->initializeOptimization();
        o->optimize(10);

        activePoses.clear();
        activeWalls.clear();
        activeEdges.clear();
    }
}
