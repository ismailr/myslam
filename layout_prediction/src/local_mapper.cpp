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

LocalMapper2::LocalMapper2(System2& system, Graph2& graph)
    :_graph (&graph), _system (&system)
{
    _system->set_local_mapper (*this);
    _optimizer = new g2o::SparseOptimizer();

    typedef BlockSolver<BlockSolverTraits<-1,-1> > SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering (false);
    OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton (
            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    _optimizer->setAlgorithm (solver);
    _optimizer->setVerbose (true);
}

Wall2::Ptr LocalMapper2::data_association (Wall2::Ptr& wall)
{
    if (_wallDB.empty())
    {
        std::tuple<int,int> v = std::make_tuple (0,0);
        add_vertex (wall);
        _indexedWallDB[v] = wall;

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

    if (_indexedWallDB.count(v) == 0)
    {
        add_vertex (wall);
        _indexedWallDB[v] = wall;
        return wall;
    }
    else if (_indexedWallDB.count(v))
        return _indexedWallDB[v];
}

void LocalMapper2::local_optimize()
{
    for (std::vector<Pose2::Ptr>::iterator pit = _poseDB.begin();
            pit != _poseDB.end(); ++pit)
        _optimizer->addVertex ((*pit).get());

    for (std::vector<Wall2::Ptr>::iterator wit = _wallDB.begin();
            wit != _wallDB.end(); ++wit)
        _optimizer->addVertex ((*wit).get());

    for (std::vector<PoseMeasurement2::Ptr>::iterator pmit = _poseMeasurementDB.begin();
            pmit != _poseMeasurementDB.end(); ++pmit)
        _optimizer->addEdge ((*pmit).get());

    for (std::vector<WallMeasurement2::Ptr>::iterator wmit = _wallMeasurementDB.begin();
            wmit != _wallMeasurementDB.end(); ++wmit)
        _optimizer->addEdge ((*wmit).get());

    std::cout << "SET FIXED VERTICES ... " << std::endl;
    set_fixed_vertices();
    std::cout << "INITIALIZE OPTIMIZATION ... " << std::endl;
    std::cout << _optimizer->initializeOptimization() << std::endl;
    std::cout << "DONE INIT ... " << std::endl;
    _optimizer->optimize (10);
    std::cout << "DONE OPTIMIZATION ... " << std::endl;
    push_to_graph();
    clear();
}

void LocalMapper2::add_vertex (Pose2::Ptr& pose)
{
    int id = _system->requestUniqueId();
    std::cout << "REGISTER ID: " << id << " FOR POSE" << std::endl;
    pose->setId (id);
    _poseDB.push_back (pose);
}

void LocalMapper2::add_vertex (Wall2::Ptr& wall)
{
    int id = _system->requestUniqueId();
    wall->setId (id);
    _wallDB.push_back (wall);
}

void LocalMapper2::add_edge (PoseMeasurement2::Ptr& poseMeasurement)
{
    _poseMeasurementDB.push_back (poseMeasurement);
}

void LocalMapper2::add_edge (WallMeasurement2::Ptr& wallMeasurement)
{
    _wallMeasurementDB.push_back (wallMeasurement);
}

void LocalMapper2::set_fixed_vertices()
{
    Pose2::Ptr anchoredPose = _poseDB.front();
    anchoredPose->setFixed (true);

    if (_poseDB.front()->id() != 0)
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
    std::cout << "READY TO CLEAR POSEDB ..." << std::endl;
    Pose2::Ptr pose = _poseDB.back();
    std::cout << "FIRST GETTING LAST POSE -> ID: " << pose->id() << std::endl;
    _poseDB.clear();
    _poseDB.push_back (pose);
    std::cout << "POSE ID: " << pose->id() << std::endl;
    std::cout << "DONE CLEAR POSEDB ..." << std::endl;

    // Reset _indexedWallDB
    std::cout << "READY TO CLEAR INDEXED_WALL_DB ..." << std::endl;
    _indexedWallDB.clear();
    std::cout << "DONE CLEAR INDEXED_WALL_DB ..." << std::endl;

    // Reset _wallDB
    std::cout << "READY TO CLEAR WALL_DB ..." << std::endl;
    _wallDB.clear();
    std::cout << "WALL_DB CLEARED..." << std::endl;
    std::vector<int> walls = pose->get_detected_walls();
    std::cout << "GET WALLS DETECTED FROM POSE WITH ID " << pose->id() << std::endl;
    for (std::vector<int>::iterator it = walls.begin();
            it != walls.end(); ++it)
    {
        std::cout << "TRYING TO RECOVERED WALL FROM OPTIMIZER " << *it << std::endl;
        Wall2* w = dynamic_cast<Wall2*>(_optimizer->vertex(*it));
        std::cout << "DONE" << std::endl;
        Wall2::Ptr wPtr (w);
        std::cout << "GET WALL ID " << w->id() << std::endl;

        std::cout << "TRYING TO REFILL WALLDB WITH WALL ID " << w->id() << std::endl;
        if (_wallDB.empty())
        {
            std::tuple<int,int> v = std::make_tuple (0,0);
            _wallDB.push_back (wPtr);
            _indexedWallDB[v] = wPtr;
            continue;
        }

        Wall2::Ptr refWall = _wallDB.front();

        double rho_ref = refWall->rho();
        double theta_ref = refWall->theta();

        double rho = wPtr->rho();
        double theta = wPtr->theta();

        int rho_index = round (std::abs (rho - rho_ref)/GRID_STEP);
        int theta_index = round (std::abs (theta - theta_ref)/ANGLE_STEP);

        std::tuple<int,int> v = std::make_tuple (rho_index, theta_index);

        if (_indexedWallDB.count(v) == 0)
        {
            _wallDB.push_back (wPtr);
            _indexedWallDB[v] = wPtr;
        }
        std::cout << "DONE" << std::endl;
    }
    std::cout << "DONE CLEAR WALL_DB ..." << std::endl;

    std::cout << "RESET OPTIMIZER  ..." << std::endl;
    // Reset _optimizer
    _optimizer->clear();
}
