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

unsigned long Graph::id = 0;

Graph::Graph (){}

void Graph::addVertex (Wall::Ptr wallPtr)
{
    _walls[wallPtr->id()] = wallPtr;
    _wallIds.push_back (wallPtr->id());
}

void Graph::addVertex (Pose::Ptr posePtr)
{
    _poses[posePtr->id()] = posePtr;
    _poseIds.push_back (posePtr->id());
}

void Graph::addEdgePose2Pose (std::tuple<int, int> e, SE2& t)
{
    PoseMeasurement::Ptr poseMeasurementPtr (new PoseMeasurement);
    poseMeasurementPtr.get()->vertices()[0] = getPoseVertex (std::get<0>(e)).get();
    poseMeasurementPtr.get()->vertices()[1] = getPoseVertex (std::get<1>(e)).get();
    poseMeasurementPtr.get()->setMeasurement (t);
    Eigen::Matrix<double, 3, 3> inf;
    inf.setIdentity();
    poseMeasurementPtr.get()->information () = inf;
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
    typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    g2o::SparseOptimizer graph;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering (false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver (linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton (blockSolver);
    graph.setAlgorithm (solver);

    std::vector<int>::iterator pose_it = std::find (_poseIds.begin(), _poseIds.end(), poseId);

//    for (int i = 3; i >= 0; i--)
//    {
//        // if anchored vertices
//        if (i == 0)
//            setAnchor (*pose_it, true);
//
//        // add pose vertices
//        std::cout << "trying to add vertex pose of id: " << *pose_it << std::endl;
//        graph.addVertex (_poses[*pose_it].get());
//        std::cout << "success.." << std::endl;
//
//        // add wall vertices and pose2wall edges
//        for (std::vector<int>::iterator wall_it = _pose2wallmap[*pose_it].begin(); 
//                wall_it != _pose2wallmap[*pose_it].end(); wall_it++)
//        {
//            std::cout << "trying to add vertex wall of id: " << *wall_it << std::endl;
//            graph.addVertex (_walls[*wall_it].get());
//            std::cout << "success.." << std::endl;
//            graph.addEdge (_pose2wall[std::tuple<int, int> (*pose_it, *wall_it)].get());
//        }
//
//        // add edge between poses
//        if (i != 0)
//            graph.addEdge (_pose2pose[std::tuple<int, int> (*(pose_it - 1), *pose_it)].get());
//
//        pose_it--;
//    }

//    graph.save ("/home/ism/local_before.g2o");
//    graph.initializeOptimization();
//    graph.optimize (10);
//    graph.save ("/home/ism/local_after.g2o");
//    graph.clear();

//    Factory::destroy();
//    OptimizationAlgorithmFactory::destroy();
//    HyperGraphActionLibrary::destroy();

//    setAnchor (*pose_it, false);
    occupancyGrid (poseId);
}

void Graph::setAnchor (int poseId, bool set)
{
    _poses[poseId]->setFixed (set);
    for (std::vector<int>::iterator it = _pose2wallmap[poseId].begin();
            it != _pose2wallmap[poseId].end(); it++)
    {
        _walls[*it]->setFixed (set);
    }
}

void Graph::occupancyGrid (int poseId)
{
    std::vector<int>::reverse_iterator it = std::find (_poseIds.rbegin(), _poseIds.rend(), poseId);

    int id_ref; // pose reference
    it + 3 == _poseIds.rbegin() ? id_ref = *(it + 2) : id_ref = *(it + 3);

    // walls as references
    std::vector<int> wall_ref = _pose2wallmap [id_ref];

    for (std::vector<int>::reverse_iterator pose_it = it;
            pose_it != it + 4; pose_it++) // for every pose in poseId, poseId-1, poseId-2, poseId-3 ...
    {
        if (pose_it == _poseIds.rend()) continue;

        std::cout << "pose: " << *pose_it << std::endl;
        for (std::vector<int>::iterator wall_it = _pose2wallmap[*pose_it].begin();
                wall_it != _pose2wallmap[*pose_it].end(); wall_it++)    // ... match wall detected in that pose to wall_ref
        {
            std::cout << *wall_it << " --> ";
            *wall_it = associateData (*wall_it, wall_ref); // rewire pose from which *wall_it observed to wall \w id match
            std::cout << *wall_it << std::endl;
        }
        std::cout << "-------------------------" << std::endl;
    }
}

int Graph::associateData (int wallId, std::vector<int> ref)
{
    double rho = _walls[wallId]->rho();
    double theta = _walls[wallId]->theta();

    for (std::vector<int>::iterator it = ref.begin();
            it != ref.end(); it++)
    {
        double rho_ref = _walls[*it]->rho();
        double theta_ref = _walls[*it]->theta();

        if (    std::abs (rho - rho_ref) < 5.0 &&
                std::abs (theta - theta_ref) < 22.5 )
            return *it;
    }

    return wallId;
}
