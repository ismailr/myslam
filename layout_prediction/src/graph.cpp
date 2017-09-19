#include <math.h>
#include <fstream>
#include <iostream>
#include <cmath>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include "layout_prediction/graph.h"
#include "layout_prediction/wall.h"

using namespace g2o;

Graph::Graph (){}

std::vector<Wall::Ptr> Graph::getAllVertices ()
{
    return _walls;
}

void Graph::addEdgePose2Pose (std::tuple<int, int> e, SE2& t)
{
    PoseMeasurement::Ptr poseMeasurementPtr (new PoseMeasurement);
    poseMeasurementPtr.get()->vertices()[0] = getPoseVertex (std::get<0>(e)).get();
    poseMeasurementPtr.get()->vertices()[1] = getPoseVertex (std::get<1>(e)).get();
    poseMeasurementPtr.get()->setMeasurement (t);
    //poseMeasurementPtr.get()->setInformation ();
   _pose2pose[e] = poseMeasurementPtr; 
}

void Graph::addEdgePose2Wall (std::tuple<int, int> e, double* m)
{
    WallMeasurement::Ptr wallMeasurementPtr (new WallMeasurement);
    wallMeasurementPtr.get()->vertices()[0] = getPoseVertex (std::get<0>(e)).get();
    wallMeasurementPtr.get()->vertices()[1] = getWallVertex (std::get<1>(e)).get();
    wallMeasurementPtr->setMeasurementData (m);
    _pose2wall[e] = wallMeasurementPtr;
    _pose2wallmap[std::get<0>(e)].push_back(std::get<1>(e));
}

void Graph::updateGraph (std::vector<Wall::Ptr> newGraph)
{
    std::unique_lock<std::mutex> lock (_graphUpdateMutex);
    for (int i = 0; i < newGraph.size(); ++i)
        // _walls[newGrah[i].id] = newGraph[i];
    lock.unlock();
}

bool Graph::localOptimize (int poseId)
{
    if (poseId < 4) return false;

    typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    g2o::SparseOptimizer graph;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering (false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver (linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton (blockSolver);
    graph.setAlgorithm (solver);

    std::cout << "poseId: " << poseId << std::endl;

    for (int i = poseId - 3; i <= poseId; i++)
    {
        // if anchored vertices
        if (i == poseId - 3)
            setAnchor (poseId - 3, true);

        // add pose vertices
        std::cout << "trying to add vertex pose of id: " << i << std::endl;
        graph.addVertex (_posemap[i].get());
        std::cout << "success.." << std::endl;

        // add wall vertices and pose2wall edges
        for (std::vector<int>::iterator it = _pose2wallmap[i].begin(); 
                it != _pose2wallmap[i].end(); it++)
        {
            std::cout << "trying to add vertex wall of id: " << *it << std::endl;
            graph.addVertex (_wallmap[*it].get());
            std::cout << "success.." << std::endl;
            graph.addEdge (_pose2wall[std::tuple<int, int> (i, *it)].get());
        }

        // add edge between poses
        if (i < poseId)
            graph.addEdge (_pose2pose[std::tuple<int, int> (i, i + 1)].get());
    }

    graph.save ("/home/ism/local_before.g2o");
    graph.initializeOptimization();
    graph.optimize (10);
    graph.save ("/home/ism/local_after.g2o");
    graph.clear();

    Factory::destroy();
    OptimizationAlgorithmFactory::destroy();
    HyperGraphActionLibrary::destroy();

    setAnchor (poseId - 3, false);
}

void Graph::setAnchor (int poseId, bool set)
{
    _posemap[poseId]->setFixed (set);
    for (std::vector<int>::iterator it = _pose2wallmap[poseId].begin();
            it != _pose2wallmap[poseId].end(); it++)
    {
        _wallmap[*it]->setFixed (set);
    }
}
