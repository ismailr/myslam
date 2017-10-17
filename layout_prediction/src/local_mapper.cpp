#include "layout_prediction/local_mapper.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/wall_measurement.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/hyper_graph.h>

using namespace g2o;

int LocalMapper::frame_counter = 0;
LocalMapper::LocalMapper (Graph& graph): _graph (&graph) { }
void LocalMapper::optimize (std::vector<Wall::Ptr> walls) { }
void LocalMapper::occupancyGrid () { }

LocalMapper2::LocalMapper2(System2& system, Graph2& graph)
    :_graph (&graph), _system (&system)
{
    _system->set_local_mapper (*this);
}

Wall2::Ptr LocalMapper2::data_association (Wall2::Ptr& wall)
{
    if (_wallDB.empty())
    {
        std::tuple<int,int> v = std::make_tuple (0,0);
        _graph->registerWall (wall);
        pushWall (wall);
        _localGrid[v] = wall;

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

    if (_localGrid.count(v) == 0)
    {
        _graph->registerWall (wall);
        pushWall (wall);
        _localGrid[v] = wall;
        return wall;
    }
    else if (_localGrid.count(v))
        return _localGrid[v];
}

void LocalMapper2::optimize()
{
    g2o::HyperGraph::VertexSet vertexSet;
    g2o::HyperGraph::EdgeSet edgeSet;

    std::cout << "POSE TO OPTIMIZE: ";
    for (int i = 0; i < _poseDB.size(); i++) {
        vertexSet.insert (_poseDB[i].get());
        std::cout << _poseDB[i]->id() << ", ";
    }
    std::cout << std::endl;

    std::cout << "WALL TO OPTIMIZE: ";
    for (int i = 0; i < _wallDB.size(); i++) {
        vertexSet.insert (_wallDB[i].get());
        std::cout << _wallDB[i]->id() << ", ";
    }
    std::cout << std::endl;

    std::cout << "POSE-POSE EDGE TO OPTIMIZE: ";
    for (int i = 0; i < _poseMeasurementDB.size(); i++) {
        edgeSet.insert (_poseMeasurementDB[i].get());
        std::cout << _poseMeasurementDB[i]->id() << ", ";
    }
    std::cout << std::endl;

    std::cout << "POSE-WALL EDGE TO OPTIMIZE: ";
    for (int i = 0; i < _wallMeasurementDB.size(); i++) {
        edgeSet.insert (_wallMeasurementDB[i].get());
        std::cout << _wallMeasurementDB[i]->id() << ", ";
    }
    std::cout << std::endl;

//    set_fixed_vertices();
    clear();
}

void LocalMapper2::set_fixed_vertices()
{
    Pose2::Ptr anchoredPose = _poseDB.front();
    anchoredPose->setFixed (true);

    if (_poseDB.front()->id() != 0)
    {
        std::vector<Wall2::Ptr> walls = anchoredPose->get_detected_walls();
        for (std::vector<Wall2::Ptr>::iterator it = walls.begin();
                it != walls.end(); it++)
        {
            (*it)->setFixed (true);
        }
    }
}

void LocalMapper2::push_to_graph (){}
void LocalMapper2::clear()
{
    // Reset _poseDB
    Pose2::Ptr pose = _poseDB.back();
    _poseDB.clear();
    _poseDB.push_back (pose);

    // Reset _indexedWallDB
    _localGrid.clear();

    // Reset _wallDB
    _wallDB.clear();
    std::vector<Wall2::Ptr> walls = pose->get_detected_walls();
    for (std::vector<Wall2::Ptr>::iterator it = walls.begin();
            it != walls.end(); ++it)
    {
        if (_wallDB.empty())
        {
            std::tuple<int,int> v = std::make_tuple (0,0);
            _wallDB.push_back (*it);
            _localGrid[v] = *it;
            continue;
        }

        Wall2::Ptr refWall = _wallDB.front();

        double rho_ref = refWall->rho();
        double theta_ref = refWall->theta();

        double rho = (*it)->rho();
        double theta = (*it)->theta();

        int rho_index = round (std::abs (rho - rho_ref)/GRID_STEP);
        int theta_index = round (std::abs (theta - theta_ref)/ANGLE_STEP);

        std::tuple<int,int> v = std::make_tuple (rho_index, theta_index);

        if (_localGrid.count(v) == 0)
        {
            _wallDB.push_back (*it);
            _localGrid[v] = *it;
        }
    }

    // reset poseMeasurementDB and wallMeasurementDB
    _poseMeasurementDB.clear();
    _wallMeasurementDB.clear();
}
