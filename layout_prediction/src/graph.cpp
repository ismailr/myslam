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
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/parameter_se2_offset.h>

#include "layout_prediction/graph.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/settings.h"

using namespace g2o;

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
//    OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton (
//            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg (
            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    _optimizer->setAlgorithm (solver);
    _optimizer->setVerbose (true);

    _incOptimizer = new g2o::SparseOptimizerIncremental;
    _incOptimizer->setVerbose (true);

    _slamInterface = new g2o::G2oSlamInterface (_incOptimizer);
    _slamInterface->setUpdateGraphEachN (10);
    _slamInterface->setBatchSolveEachN (100);
}

Pose2::Ptr Graph2::createPose()
{
    Pose2::Ptr p (new Pose2);
    _poseDB.push_back (p);
    _poseMap[p->id()] = p;
    return p;
}

Pose2::Ptr Graph2::createPoseWithId()
{
    Pose2::Ptr p (new Pose2);
    p->setId (requestId());
    _poseDB.push_back (p);
    _poseMap[p->id()] = p;
    return p;
}

Wall2::Ptr Graph2::createWall()
{
    Wall2::Ptr w (new Wall2);
    return w;
}

Wall3::Ptr Graph2::createWall3()
{
    Wall3::Ptr w (new Wall3);
    return w;
}

Wall2::Ptr Graph2::createWallWithId()
{
    Wall2::Ptr w (new Wall2);
    w->setId (requestId());
    return w;
}

Wall3::Ptr Graph2::createWall3WithId()
{
    Wall3::Ptr w (new Wall3);
    w->setId (requestId());
    return w;
}

void Graph2::registerWall (Wall2::Ptr& wall)
{
    _wallDB.push_back (wall);
}

void Graph2::registerWallWithId (Wall2::Ptr& wall)
{
    wall->setId (requestId());
    _wallDB.push_back (wall);
}

void Graph2::registerWall3 (Wall3::Ptr& wall)
{
    _wallDB3.push_back (wall);
    _wallMap[wall->id()] = wall;
}

PoseMeasurement2::Ptr Graph2::createPoseMeasurement()
{
    PoseMeasurement2::Ptr pm (new PoseMeasurement2);
    _poseMeasurementDB.push_back (pm);
    return pm;
}

PoseMeasurement2::Ptr Graph2::createPoseMeasurement(int from, int to)
{
    PoseMeasurement2::Ptr pm (new PoseMeasurement2);
    _poseMeasurementDB.push_back (pm);
    _pose2poseMap[std::tuple<int,int>(from,to)] = pm;
    return pm;
}

WallMeasurement2::Ptr Graph2::createWallMeasurement()
{
    WallMeasurement2::Ptr wm (new WallMeasurement2);
    _wallMeasurementDB.push_back (wm);
    return wm;
}

WallMeasurement3::Ptr Graph2::createWallMeasurement3()
{
    WallMeasurement3::Ptr wm (new WallMeasurement3);
    _wallMeasurementDB3.push_back (wm);
    return wm;
}

WallMeasurement3::Ptr Graph2::createWallMeasurement3(int from, int to)
{
    WallMeasurement3::Ptr wm (new WallMeasurement3);
    _wallMeasurementDB3.push_back (wm);
    _pose2wallMap[std::tuple<int,int>(from,to)] = wm;
    return wm;
}

//AngleMeasurement::Ptr Graph2::createAngleMeasurement()
//{
//    AngleMeasurement::Ptr am (new AngleMeasurement);
//    _angleMeasurementDB.push_back (am);
//    return am;
//}

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
    else
        return _grid[v];
//    else if (_grid.count(v))
//    {
//        Eigen::Vector2d p = _grid[v]->get_center_point();
//        Eigen::Vector2d q = wall->get_center_point();
//        double d = calculate_euclidean_distance (p,q);
//
//        if (d > CENTER_THRESHOLD)
//        {
//            registerWall (wall);
//            _grid[v] = wall;
//            return wall;
//        } 
//        else
//        {
//            return _grid[v];
//        }
//    }
}

Wall3::Ptr Graph2::data_association (Wall3::Ptr& wall)
{
    if (_wallDB3.empty())
    {
        registerWall3 (wall);
        return wall;
    }

    double xest = wall->estimate().x();
    double yest = wall->estimate().y();

    for (std::vector<Wall3::Ptr>::iterator it = _wallDB3.begin(); 
            it != _wallDB3.end(); it++)
    {
        if (    std::abs(xest - (*it)->estimate().x()) < THRESHOLD &&
                std::abs(yest - (*it)->estimate().y()) < THRESHOLD)
        {
            return *it;
        }
    }

    registerWall3 (wall);
    return wall;
}

static int iter = 0;
void Graph2::optimize()
{
//    SE2 o (29.745491, 129.72481321, 0.3098093);
//    ParameterSE2Offset *offset = new ParameterSE2Offset;
//    offset->setOffset (o);
//    offset->setId (0);
//    _optimizer->addParameter (offset);

    g2o::HyperGraph::VertexSet vertexSet;
    g2o::HyperGraph::EdgeSet edgeSet;

    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin() + _pid;
            it != _poseDB.end(); ++it)
    {
        int id = requestId();
        (*it)->setId (id);
        if (it == _poseDB.begin()) (*it)->setFixed (true);
//        vertexSet.insert ((*it).get());
//        edgeSet.insert ((*it)->edges().begin(), (*it)->edges().end());
        _optimizer->addVertex ((*it).get());
//        _pid++;
    }

//    for (std::vector<Wall2::Ptr>::iterator it = _wallDB.begin() + _wid;
//            it != _wallDB.end(); ++it)
//    {
//        int id = requestId();
//        (*it)->setId (id);
//        if (it == _wallDB.begin() + 3) (*it)->setFixed (true);
//        vertexSet.insert ((*it).get());
//        edgeSet.insert ((*it)->edges().begin(), (*it)->edges().end());
//        _optimizer->addVertex ((*it).get());
//        _wid++;

//        if (it !=_wallDB.begin())
//        {
//            AngleMeasurement::Ptr am = createAngleMeasurement();
//            am->vertices()[0] = (*(it - 1)).get();
//            am->vertices()[1] = (*it).get();
//            _optimizer->addEdge (am.get());
//            Eigen::Matrix<double, 2, 2> inf;
//            inf.setIdentity();
//            am->information () = inf;
//        }

//    }

    for (std::vector<Wall3::Ptr>::iterator it = _wallDB3.begin() + _wid;
            it != _wallDB3.end(); ++it)
    {
        int id = requestId();
        (*it)->setId (id);
//        if (it == _wallDB.begin() + 3) (*it)->setFixed (true);
//        vertexSet.insert ((*it).get());
//        edgeSet.insert ((*it)->edges().begin(), (*it)->edges().end());
        _optimizer->addVertex ((*it).get());
//       _wid++;

//        if (it !=_wallDB.begin())
//        {
//            AngleMeasurement::Ptr am = createAngleMeasurement();
//            am->vertices()[0] = (*(it - 1)).get();
//            am->vertices()[1] = (*it).get();
//            _optimizer->addEdge (am.get());
//            Eigen::Matrix<double, 2, 2> inf;
//            inf.setIdentity();
//            am->information () = inf;
//        }

    }

    for (std::vector<PoseMeasurement2::Ptr>::iterator it = _poseMeasurementDB.begin() + _pmid;
            it != _poseMeasurementDB.end(); ++it)
    {
        _optimizer->addEdge ((*it).get());
        _pmid++;
    }

//    for (std::vector<WallMeasurement2::Ptr>::iterator it = _wallMeasurementDB.begin() + _wmid;
//            it != _wallMeasurementDB.end(); ++it)
//    {
//        _optimizer->addEdge ((*it).get());
//        _wmid++;
//    }

    for (std::vector<WallMeasurement3::Ptr>::iterator it = _wallMeasurementDB3.begin() + _wmid;
            it != _wallMeasurementDB3.end(); ++it)
    {
        _optimizer->addEdge ((*it).get());
        _wmid++;
    }


    if (_pid > 0)
    {
        std::vector<Pose2::Ptr>::iterator it = _poseDB.begin() + _pid;
        vertexSet.insert ((*it).get());
        edgeSet.insert ((*it)->edges().begin(), (*it)->edges().end());
        for (g2o::HyperGraph::EdgeSet::iterator eit = edgeSet.begin(); eit != edgeSet.end(); eit++)
        {
            Wall3* from = dynamic_cast<Wall3*>((*eit)->vertices()[0]);
            Wall3* to = dynamic_cast<Wall3*>((*eit)->vertices()[1]);
            if (from && from->nodetype == "WALL3")
            {
                from->setFixed (true);
            }
            if (to && to->nodetype == "WALL3")
            {
                to->setFixed (true);
            }
        }
        
    }

    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin() + _pid;
            it != _poseDB.end(); ++it)
    {
        if (it == _poseDB.begin() + _pid)
            (*it)->setFixed (true);
        vertexSet.insert ((*it).get());
        edgeSet.insert ((*it)->edges().begin(), (*it)->edges().end());
        _pid++;
    }

    for (std::vector<Wall3::Ptr>::iterator it = _wallDB3.begin() + _wid;
            it != _wallDB3.end(); ++it)
    {
        vertexSet.insert ((*it).get());
        edgeSet.insert ((*it)->edges().begin(), (*it)->edges().end());
        _wid++;
    }


//    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin();
//            it != _poseDB.end(); ++it)
//    {
//        myfile << "POSE " << (*it)->id() << " "; 
//        (*it)->write (myfile);
//        myfile << std::endl;
//    }

//    std::ofstream myfile;
//    myfile.open ("/home/ism/tmp/wall.dat", std::ios::out | std::ios::app);
//    myfile << "BEFORE" << std::endl;
//    for (std::vector<Wall2::Ptr>::iterator it = _wallDB.begin();
//            it != _wallDB.end(); ++it)
//    {
//        myfile << "WALL " << (*it)->id() << " ";
//        (*it)->write (myfile);
//        myfile << std::endl;
//    }

    if (iter == 0)
    {
        _optimizer->initializeOptimization();
        iter++;
    }
    else
    {
        _optimizer->updateInitialization (vertexSet, edgeSet);
    }

    _optimizer->optimize (10);
    vertexSet.clear();
    edgeSet.clear();

//    myfile << "AFTER" << std::endl;
//    for (std::vector<Wall2::Ptr>::iterator it = _wallDB.begin();
//            it != _wallDB.end(); ++it)
//    {
//        myfile << "WALL " << (*it)->id() << " ";
//        (*it)->write (myfile);
//        myfile << std::endl;
//    }
//    myfile.close();

    std::ofstream posefile;
    posefile.open ("/home/ism/tmp/finalpose.dat", std::ios::out);

    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin();
            it != _poseDB.end(); ++it)
    {
        posefile << (*it)->estimate().toVector()(0) << " ";
        posefile << (*it)->estimate().toVector()(1) << " ";
        posefile << (*it)->estimate().toVector()(2) << std::endl; 
    }
    posefile.close();
    
}

double Graph2::calculate_euclidean_distance (Eigen::Vector2d p, Eigen::Vector2d q)
{
    double d = sqrt ((p(0) - q(0)) * (p(0) - q(0)) + (p(1) - q(1)) * (p(1) - q(1)));
    return d;
}

void Graph2::localOptimize(bool init)
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

    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin(); 
            it != _poseDB.end(); ++it)
    {
//        int id = requestId();
//        (*it)->setId (id);

        if (init)
        {
            o->addVertex ((*it).get());
            if ((*it)->id() == 0) (*it)->setFixed (true);
        }
        else if (it != _poseDB.begin())
            o->addVertex ((*it).get());
        else (*it)->setFixed (true);
    }

    for (std::vector<Wall3::Ptr>::iterator it = _wallDB3.begin();
            it != _wallDB3.end(); ++it)
    {
//        int id = requestId();
//        (*it)->setId (id);
        o->addVertex ((*it).get());

//        Pose2::Ptr pose = *_poseDB.begin();
//        if (pose->is_detected_wall ((*it)->id()))
//            (*it)->setFixed (true);

//        if (it !=_wallDB3.begin())
//        {
//            AngleMeasurement::Ptr am = createAngleMeasurement();
//            am->vertices()[0] = (*(it - 1)).get();
//            am->vertices()[1] = (*it).get();
//            o->addEdge (am.get());
//            Eigen::Matrix<double, 1, 1> inf;
//            inf.setIdentity();
//            am->information () = inf;
//        }
    }

    for (std::vector<PoseMeasurement2::Ptr>::iterator it = _poseMeasurementDB.begin();
            it != _poseMeasurementDB.end(); ++it)
    {
//        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//        ((*it).get())->setRobustKernel (rk);
//        rk->setDelta (sqrt(5.99));
        o->addEdge ((*it).get());
    }

    for (std::vector<WallMeasurement3::Ptr>::iterator it = _wallMeasurementDB3.begin();
            it != _wallMeasurementDB3.end(); ++it)
    {
//        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//        ((*it).get())->setRobustKernel (rk);
//        rk->setDelta (sqrt(5.99));
        o->addEdge ((*it).get());
    }

    std::ofstream mfile;
    mfile.open ("/home/ism/tmp/data.g2o", std::ios::out | std::ios::app);
    o->save(mfile);
    o->initializeOptimization();
    o->optimize(100);
    o->save(mfile);
    mfile << std::endl << std::endl;
    mfile.close();

    std::ofstream posefile;
    posefile.open ("/home/ism/tmp/finalpose.dat", std::ios::out | std::ios::app);

    for (std::vector<Pose2::Ptr>::iterator it = _poseDB.begin();
            it != _poseDB.end(); ++it)
    {
        if (!init && it == _poseDB.begin()) continue;
        posefile << (*it)->estimate().toVector()(0) << " ";
        posefile << (*it)->estimate().toVector()(1) << " ";
        posefile << (*it)->estimate().toVector()(2) << std::endl; 
    }
    posefile.close();
//    _system->visualize<Wall3::Ptr> (_wall3DB);

    Pose2::Ptr p = _poseDB.back();
    std::vector<Wall3::Ptr> wallFixed;
    for (std::vector<Wall3::Ptr>::iterator it = _wallDB3.begin();
            it != _wallDB3.end(); it++)
    {
        if (p->is_detected_wall ((*it)->id()))
        {
            Wall3 *w = dynamic_cast<Wall3*> ((*it)->clone());
//            Wall3::Ptr ww (w);
//            wallFixed.push_back (ww);
        }
    }
    _poseMeasurementDB.clear();
    _wallMeasurementDB3.clear();
    _poseDB.clear();
    _wallDB3.clear();
    _poseDB.push_back(p);
    _wallDB3.insert (_wallDB3.end(), wallFixed.begin(), wallFixed.end());

//    _system->getTracker()->fixLastPose (p);
}

void Graph2::localOptimize(bool init, std::map<int, std::set<int> > data)
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

    std::map<int, std::set<int> >::iterator it = data.begin();
    for (it; it != data.end(); it++)
    {
        if (!o->vertex (it->first)) o->addVertex (_poseMap[it->first].get());
        for (std::set<int>::iterator wit = it->second.begin(); wit != it->second.end(); wit++)
        {
            if (o->vertex (*wit)) o->addVertex (_wallMap[*wit].get());
            if (!init && it == data.begin() && _pose2wallMap[std::tuple<int,int>(it->first,*wit)])
                _wallMap[*wit]->setFixed (true);

            if (_pose2wallMap[std::tuple<int,int>(it->first,*wit)])
                o->addEdge (_pose2wallMap[std::tuple<int,int>(it->first,*wit)].get());
        }
        if (it == data.begin()) _poseMap[it->first]->setFixed (true);
        if (it != data.begin())
            o->addEdge(_pose2poseMap[std::tuple<int,int>(std::prev(it,1)->first, it->first)].get());
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

    for (std::map<int, Pose2::Ptr>::iterator it = _poseMap.begin();
            it != _poseMap.end(); ++it)
    {
        posefile << it->second->estimate().toVector()(0) << " ";
        posefile << it->second->estimate().toVector()(1) << " ";
        posefile << it->second->estimate().toVector()(2) << std::endl; 
    }
    posefile.close();
}

void Graph2::optimizeIncremental()
{

}

namespace MYSLAM {
    Graph::Graph(){}; // simulation
    Graph::Graph(System& system):_system (&system) {};

    Wall::Ptr Graph::dataAssociation (Wall::Ptr& w)
    {
        double& xx = w->_line.xx[0];
        double& xy = w->_line.xx[1];

        std::set<int> walls;
        walls.insert (_lastActiveWalls.begin(), _lastActiveWalls.end());
        walls.insert (_activeWalls.begin(), _activeWalls.end());

        for (std::set<int>::iterator it = walls.begin();
                it != walls.end(); it++)
        {
            double& xxref = _wallMap[*it]->_line.xx[0];
            double& xyref = _wallMap[*it]->_line.xx[1];

            if (    std::abs(xx-xxref) < MYSLAM::DATA_ASSOCIATION_THRESHOLD 
                    && std::abs(xy-xyref) < MYSLAM::DATA_ASSOCIATION_THRESHOLD)
                return _wallMap[*it];
        }

        _wallMap[w->_id] = w;
//        _activeWalls.push_back (w->_id);
        return w;
    }

    bool Graph::dataAssociationEKF (int poseid, Wall::Ptr& w, const Eigen::Vector2d& z) {

        if (_wallMap.empty()) return true;

        SE2 pose; pose.fromVector (_poseMap[poseid]->_pose);

        double pmax = 0.0;
        int id;
        Eigen::Matrix2d cov;

        // maximum likelihood
        for (std::map<int, Wall::Ptr>::iterator it = _wallMap.begin();
                it != _wallMap.end(); it++)
        {
            // sensor model
            Wall::Ptr _w = it->second;

            Eigen::Vector2d z_hat = pose.inverse() * _w->_line.xx;
            Eigen::Vector2d e = z - z_hat;

            double den = 2 * M_PI * sqrt(_w->cov.determinant());
            double num = std::exp(-0.5 * e.transpose() * _w->cov.inverse() * e);
            double p = num/den;

            std::cout << "INOV: " << e.transpose() << std::endl;
            std::cout << "DEN: " << den << std::endl;
            std::cout << "NUM: " << num << std::endl;
            std::cout << "COV: " << _w->cov.inverse() << std::endl;
            std::cout << "P: " << p << std::endl;
            std::cout << "PMAX: " << pmax << std::endl;
            if (p > pmax) {
                pmax = p;
                id = _w->_id;
                cov = _w->cov;
            }
        }

        double stdevx = sqrt (cov(0,0));
        double stdevy = sqrt (cov(1,1));
        Eigen::Vector2d stdv (stdevx, stdevy);
        double den_thres = 2 * M_PI * sqrt (cov.determinant());
        double num_thres = std::exp (-0.5 * stdv.transpose() * cov.inverse() * stdv);
        double p_thres = num_thres/den_thres;

        if (pmax > p_thres) {
            w = _wallMap[id];
            return false;
           
        } else {
            return true;
        }
    }
}
