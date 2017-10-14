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

using namespace g2o;

int LocalMapper::frame_counter = 0;

LocalMapper::LocalMapper (Graph& graph): _graph (&graph)
{

}

void LocalMapper::optimize (std::vector<Wall::Ptr> walls)
{
//    ++LocalMapper::frame_counter;
//
//    _walls.insert (_walls.end(), walls.begin(), walls.end());
//    if (LocalMapper::frame_counter % 3 == 0)
//    {
//        occupancyGrid ();
//        for (   std::vector<Wall::Ptr>::iterator it = _walls.begin();
//                it != _walls.end(); it++)
//        {
//            Pose *posePtr = (*it)->getPose().get();
//            Wall *wallPtr = (*it).get();
//            _optimizer->addVertex (posePtr);
//            _optimizer->addVertex (wallPtr);
//
//            if (it + 1 != _walls.end())
//            {
//                PoseMeasurement* poseMeasurementPtr = new PoseMeasurement ();
//                Pose *posePtr_ = (*(it + 1))->getPose().get();
//                poseMeasurementPtr->vertices()[0] = posePtr;
//                poseMeasurementPtr->vertices()[1] = posePtr_;
//                // poseMeasurementPtr->setMeasurement (SE2 odometry);
//                _optimizer->addEdge (poseMeasurementPtr);
//            }
//
//            std::vector<Pose::Ptr> _observerPoses = (*it)->getObserverPoses();
//            for (std::vector<Pose::Ptr>::iterator it_poses = _observerPoses.begin();
//                    it_poses != _observerPoses.end(); it_poses++)
//            {
//                WallMeasurement* wallMeasurementPtr = new WallMeasurement ();
//                wallMeasurementPtr->vertices()[0] = (*it_poses).get();
//                wallMeasurementPtr->vertices()[1] = wallPtr;
//                // wallMeasurementPtr->setMeasurementData (data);
//                _optimizer->addEdge (wallMeasurementPtr);
//            }
//        }
//        _walls.clear();
//    }
}

void LocalMapper::occupancyGrid ()
{
//    for (std::vector<Wall::Ptr>::iterator it = _walls.begin() + 1;
//            it != _walls.end(); it++)
//    {
//        Wall::Ptr wall_origin = _walls.front();
//        double rho_origin = wall_origin->rho();
//        double theta_origin = wall_origin->theta();
//
//        double rho = floor (std::abs ((*it)->rho() - rho_origin)/4);
//        double theta = floor (std::abs ((*it)->theta() - theta_origin)/22.5);
//
//        std::tuple<double, double> cell (rho, theta);
//        _local_map[cell].push_back (*it);
//    }
//
//    _walls.clear();
//    for (std::map<std::tuple<double, double>, std::vector<Wall::Ptr> >::iterator it = _local_map.begin();
//            it != _local_map.end(); it++)
//    {
//        for (std::vector<Wall::Ptr>::iterator it_walls = it->second.begin() + 1;
//                                                it_walls != it->second.end(); it_walls++)
//        {
//            Pose::Ptr posePtr = (*it_walls)->getPose();
//            it->second.front()->setObserverPose (posePtr);
//        }
//        _walls.push_back (it->second.front());
//    }
}

int LocalMapper2::localId = 0;
LocalMapper2::LocalMapper2(System2& system, Graph2& graph)
    :_graph (&graph), _system (&system)
{
    _system->set_local_mapper (*this);

}

Wall2::Ptr LocalMapper2::data_association (Wall2::Ptr& wall)
{
    const float GRID_STEP = 5.0;
    const float ANGLE_STEP = 30.0;

    double rho_ref = std::get<0>(_wallDatabase.begin()->first);
    double theta_ref = std::get<1>(_wallDatabase.begin()->first);

    double rho = wall->rho();
    double theta = wall->theta();

    int rho_index = round (std::abs (rho - rho_ref)/GRID_STEP);
    int theta_index = round (std::abs (theta - theta_ref)/ANGLE_STEP);

    std::tuple<double,double> v = std::make_tuple (rho_index, theta_index);

    if (_wallDatabase.empty() || _wallDatabase.count(v) == 0)
    {
        _wallDatabase[v] = wall;
        add_vertex (wall);
        return wall;
    }
    else if (_wallDatabase.count(v))
        return _wallDatabase[v];
}

void LocalMapper2::local_optimize()
{
    typedef BlockSolver<BlockSolverTraits<-1,-1> > SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering (false);
    OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton (
            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    _optimizer->setAlgorithm (solver);
    _optimizer->setVerbose (true);
    set_fixed_vertices();
    _optimizer->initializeOptimization();
    _optimizer->optimize (10);
    push_to_graph();
    clear();
}

void LocalMapper2::add_vertex (Pose2::Ptr& pose)
{
    pose->setId (LocalMapper2::localId++);
    _optimizer->addVertex (pose.get());
    _poseDB.push_back (LocalMapper2::localId);
}

void LocalMapper2::add_vertex (Wall2::Ptr& wall)
{
    wall->setId (LocalMapper2::localId++);
    _optimizer->addVertex (wall.get());
}

void LocalMapper2::add_edge (PoseMeasurement2::Ptr& poseMeasurement)
{
    _optimizer->addEdge (poseMeasurement.get());
}

void LocalMapper2::add_edge (WallMeasurement2::Ptr& wallMeasurement)
{
    _optimizer->addEdge (wallMeasurement.get());
}

void LocalMapper2::set_fixed_vertices()
{
    Pose2* anchoredPose = dynamic_cast<Pose2*>(_optimizer->vertex (_poseDB.front()));
    anchoredPose->setFixed (true);

    if (_poseDB.front() != 0)
    {
        std::vector<int> walls = anchoredPose->get_detected_walls();
        for (std::vector<int>::iterator it = walls.begin();
                it != walls.end(); it++)
            _optimizer->vertex(*it)->setFixed (true);
    }
}

void LocalMapper2::push_to_graph (){}
void LocalMapper2::clear()
{
    // Reset _poseDB
    Pose2* pose = dynamic_cast<Pose2*>(_optimizer->vertex (_poseDB.back()));
    _poseDB.clear();
    _poseDB.push_back (pose->id());

    // Reset _wallDatabase
    for (std::map<std::tuple<double,double> >, Wall2::Ptr>::iterator it = _wallDatabase.begin();
            it != _wallDatabase.end(); it++)
    {


    }

    // Reset _optimizer
}
