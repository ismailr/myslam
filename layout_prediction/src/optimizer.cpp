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

        for (std::vector<int>::iterator it = activePoses.begin();
                it != activePoses.end(); it++)
        {

        }

        for (std::vector<int>::iterator it = activeWalls.begin();
                it != activeWalls.end(); it++)
        {

        }

        for (std::vector<std::tuple<int,int> >::iterator it = activeEdges.begin();
                it != activeEdges.end(); it++)
        {

        }

        activePoses.clear();
        activeWalls.clear();
        activeEdges.clear();
    }
}
