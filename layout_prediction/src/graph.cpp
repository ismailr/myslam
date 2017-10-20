#include <math.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <string>


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
    Eigen::Matrix<double, 2, 2> inf;
    inf.setIdentity();
    wallMeasurementPtr.get()->information() = inf;
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
//    std::cout << "*************** NEW LOCAL OPTIMIZE ***********************" << std::endl;
//    typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
//    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//    g2o::SparseOptimizer graph;
//    SlamLinearSolver* linearSolver = new SlamLinearSolver();
//    linearSolver->setBlockOrdering (false);
//    SlamBlockSolver* blockSolver = new SlamBlockSolver (linearSolver);
//    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton (blockSolver);
//    graph.setAlgorithm (solver);
//    graph.setVerbose (true);
//
//    occupancyGrid (poseId);
//
//    std::vector<int>::iterator pose_it = std::find (_poseIds.begin(), _poseIds.end(), poseId);
//
//    for (int i = 2; i >= 0; i--)
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
//            std::cout << "add edge pose: " << *pose_it << " to wall: " << *wall_it << std::endl;
//            graph.addEdge (_pose2wall[std::tuple<int, int> (*pose_it, *wall_it)].get());
//            std::cout << "success.." << std::endl; 
//        }
//
//        // add edge between poses
//        if (i != 0)
//        {
//            std::cout << "add edges pose: " << *(pose_it - 1) << " to pose: " << *pose_it << std::endl;
//            graph.addEdge (_pose2pose[std::tuple<int, int> (*(pose_it - 1), *pose_it)].get());
//            std::cout << "success.." << std::endl;
//        }
//
//        pose_it--;
//    }
//
//    std::cout << "***** VERTICES in GRAPH *****" << std::endl;
//    auto v = graph.vertices();
//    for (auto vit = v.begin(); vit != v.end(); vit++)
//    {
//        std::cout << vit->first << " ";
//    }
//    std::cout << std::endl;
//
//    std::cout << "***** EDGES in GRAPH *****" << std::endl;
//    auto w = graph.edges();
//    for (auto wit = w.begin(); wit != w.end(); wit++)
//    {
//        std::cout << ((*(wit))->vertices()[0])->id() << " --> " << ((*(wit))->vertices()[1])->id() << std::endl;
//    }
//    std::cout << std::endl;
//
//    graph.initializeOptimization();
//    std::cout << "************ INITIALIZED ***** " << std::endl;
//    std::cout << "START OPTIMIZING" << std::endl;
//    graph.save ("/home/ism/local_before.g2o");
//    graph.optimize (10);
//    graph.save ("/home/ism/local_after.g2o");
//    std::cout << "SUCCESS OPTIMIZING" << std::endl;
//
//    std::cout << "READY TO CLEAR GRAPH" << std::endl;
//    graph.clear();
//    std::cout << "SUCCESS CLEARING GRAPH" << std::endl;
//
//    std::cout << "READY TO DESTROY" << std::endl;
//    Factory::destroy();
//    OptimizationAlgorithmFactory::destroy();
//    HyperGraphActionLibrary::destroy();
//
//    std::cout << "REMOVING ANCHORING" << std::endl;
//    setAnchor (*pose_it, false);
//    std::cout << "SUCCESS REMOVING ANCHORING" << std::endl;
//    std::cout << "************************ END OF LOCAL OPTIMIZE ***************" << std::endl;
}

void Graph::setAnchor (int poseId, bool set)
{
    std::cout << "UNSET FIXED VERTICES" << std::endl;

    if (_poses.find (poseId) != _poses.end())
        _poses[poseId]->setFixed (set);
    std::cout << "SUCCESS UNSET" << std::endl;
    for (std::vector<int>::iterator it = _pose2wallmap[poseId].begin();
            it != _pose2wallmap[poseId].end(); it++)
    {
        if (_walls.find (*it) != _walls.end())
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
            int return_id = associateData (*wall_it, wall_ref); // rewire pose from which *wall_it observed to wall \w id match
            if (*wall_it != return_id)
            {
                _pose2wall[std::tuple<int,int>(*pose_it,return_id)]=_pose2wall[std::tuple<int,int>(*pose_it,*wall_it)];
                _pose2wall.erase(std::tuple<int,int>(*pose_it,*wall_it));
                *wall_it = return_id;
            }
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

int Graph2::globalId = 0;
Graph2::Graph2() :
    _pid(0), _wid(0), _pmid(0), _wmid(0)
{
    _optimizer = new g2o::SparseOptimizer();
    _vertexSet = new g2o::HyperGraph::VertexSet;

    typedef BlockSolver<BlockSolverTraits<-1,-1> > SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering (false);
    OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton (
            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    _optimizer->setAlgorithm (solver);
    _optimizer->setVerbose (false);
}

Pose2::Ptr Graph2::createPose()
{
    Pose2::Ptr p (new Pose2);
    _poseDB.push_back (p);
    return p;
}

Wall2::Ptr Graph2::createWall()
{
    Wall2::Ptr w (new Wall2);
    return w;
}

void Graph2::registerWall (Wall2::Ptr& wall)
{
    _wallDB.push_back (wall);
}

PoseMeasurement2::Ptr Graph2::createPoseMeasurement()
{
    PoseMeasurement2::Ptr pm (new PoseMeasurement2);
    _poseMeasurementDB.push_back (pm);
    return pm;
}

WallMeasurement2::Ptr Graph2::createWallMeasurement()
{
    WallMeasurement2::Ptr wm (new WallMeasurement2);
    _wallMeasurementDB.push_back (wm);
    return wm;
}

Wall2::Ptr Graph2::data_association (Wall2::Ptr& wall)
{
    if (_wallDB.empty())
    {
        std::tuple<int,int> v = std::make_tuple (0,0);
        registerWall (wall);
        _grid[v] = wall;

        return wall;
    }

    Wall2::Ptr refWall = _wallDB.front();

    double rho_ref = refWall->rho();
    double theta_ref = refWall->theta();

    double rho = wall->rho();
    double theta = wall->theta();

    int rho_index = round (std::abs (rho - rho_ref)/GRID_STEP);
    int theta_index = round (std::abs (theta - theta_ref)/ANGLE_STEP);

    std::tuple<int,int> v = std::make_tuple (rho_index, theta_index);

    if (_grid.count(v) == 0)
    {
        registerWall (wall);
        _grid[v] = wall;
        return wall;
    }
    else if (_grid.count(v))
        return _grid[v];
}

static int iter = 0;
void Graph2::optimize()
{
    std::ofstream myfile;
    myfile.open ("/home/ism/tmp/data.txt", std::ios::out|std::ios::app);
    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin() + _pid;
            it != _poseDB.end(); ++it)
    {
        int id = requestId();
        (*it)->setId (id);
        if (id == 0) (*it)->setFixed (true);
        _optimizer->addVertex ((*it).get());
        _pid++;
    }

    for (std::vector<Wall2::Ptr>::iterator it = _wallDB.begin() + _wid;
            it != _wallDB.end(); ++it)
    {
        int id = requestId();
        (*it)->setId (id);
        _optimizer->addVertex ((*it).get());
        _wid++;
    }

    for (std::vector<PoseMeasurement2::Ptr>::iterator it = _poseMeasurementDB.begin() + _pmid;
            it != _poseMeasurementDB.end(); ++it)
    {
//        int id = requestId();
//        (*it)->setId (id);
        _optimizer->addEdge ((*it).get());
        _pmid++;
    }

    for (std::vector<WallMeasurement2::Ptr>::iterator it = _wallMeasurementDB.begin() + _wmid;
            it != _wallMeasurementDB.end(); ++it)
    {
//        int id = requestId();
//        (*it)->setId (id);
        _optimizer->addEdge ((*it).get());
        _wmid++;
    }

    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin();
            it != _poseDB.end(); ++it)
    {
        myfile << "POSE " << (*it)->id() << " "; 
        (*it)->write (myfile);
        myfile << std::endl;
    }

    for (std::vector<Wall2::Ptr>::iterator it = _wallDB.begin();
            it != _wallDB.end(); ++it)
    {
        myfile << "WALL " << (*it)->id() << " ";
        (*it)->write (myfile);
        myfile << std::endl;
    }

    _optimizer->initializeOptimization();
    _optimizer->optimize (10);

    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin();
            it != _poseDB.end(); ++it)
    {
        myfile << "POSE' " << (*it)->id() << " "; 
        (*it)->write (myfile);
        myfile << std::endl;
    }

    for (std::vector<Wall2::Ptr>::iterator it = _wallDB.begin();
            it != _wallDB.end(); ++it)
    {
        myfile << "WALL' " << (*it)->id() << " ";
        (*it)->write (myfile);
        myfile << std::endl;
    }

    myfile << "****************************************************" << std::endl << std::endl;

    myfile.close();
}
