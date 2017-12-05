#include <thread>
#include <random>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_ros/point_cloud.h>

#include "layout_prediction/settings.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/system.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/simulator.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/wall_measurement.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/parameter_se2_offset.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

using namespace std;
using namespace g2o;
using namespace Eigen;

Eigen::Vector2d calc_global (std::vector<Eigen::Vector2d> points, Pose2::Ptr& pose);

template<typename T> T gaussian_generator (T mean, T dev)
{
    random_device rd;
    mt19937 mt(rd());
    normal_distribution<double> dist(mean, dev);
    return dist(mt);
}

template<typename T> T uniform_generator (T min, T max)
{
    random_device rd;
    mt19937 mt(rd());
    uniform_real_distribution<double> dist(min, max);
    return dist(mt);
}

int main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    // Define system ...
//    Graph graph;
//    System system (nh, graph);
//    WallDetector wallDetector (system, graph);
//    Optimizer optimizer (system, graph);
//    Tracker tracker (system, graph);

    Graph2 graph2;
    System2 system2 (nh, graph2);
    LocalMapper2 localMapper2 (system2, graph2);
    WallDetector2 wallDetector2 (system2, graph2, localMapper2);
    Tracker2 tracker2 (system2, graph2, localMapper2);
    Simulator simulator;

    // initialize threads
//    std::thread wall_detector_thread (&WallDetector::run, wallDetector);
//    std::thread optimization_thread (&Graph2::optimize, graph2);
//    std::thread tracker_thread (&Tracker::run, tracker);

    // Get sensors data
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub (nh, "cloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub (nh, "rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub (nh, "depth", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub (nh, "odometry", 1);
    message_filters::Subscriber<nav_msgs::Odometry> action_sub (nh, "action", 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> odomcombined_sub (nh, "odomcombined", 1);

    typedef message_filters::sync_policies::ApproximateTime
        <   sensor_msgs::PointCloud2, 
            sensor_msgs::Image, 
            sensor_msgs::Image, 
            nav_msgs::Odometry, 
//            nav_msgs::Odometry,
            geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10),    cloud_sub, 
                                                                            rgb_sub, 
                                                                            depth_sub, 
                                                                            odometry_sub, 
//                                                                            action_sub,
                                                                            odomcombined_sub);

    sync.registerCallback (boost::bind (&System2::readSensorsData, &system2, _1, _2, _3, _4, _5/*, _6*/));

    ros::spin();

//    graph2.optimize();    
//    ros::Rate rate(30.0);
//    bool init = true;
//    Pose2::Ptr lastPose; 
//    int frame = 0;
//
//    Eigen::Matrix<double, 3, 3> pose_cov;
//    Eigen::Matrix<double, 2, 2> wall_cov;
//
//    double xcov = 0.5 * 0.5;
//    double ycov = 0.5 * 0.5;
//    double pcov = (10 * M_PI/180) * (10 * M_PI/180);
//    double rcov = (0.5 * 0.5);
//    double tcov = (0.1 * M_PI/180) * (0.1 * M_PI/180);
//
//    pose_cov << xcov, 0, 0,
//                0, ycov, 0,
//                0, 0, pcov;
//
//    wall_cov << rcov, 0,
//                0, tcov;
//
//    while (ros::ok())
//    {
//        if (init)
//        {
//            SE2 *pose = simulator.robot->simPose;
//            SE2 *truth = simulator.robot->truePose;
//
//            lastPose = graph2.createPose();
//            lastPose->setEstimate (*pose);  
//            init = false;
//            std::ofstream truthfile;
//            truthfile.open ("/home/ism/tmp/truth.dat", std::ios::out | std::ios::app);
//            truthfile << truth->toVector().transpose() << std::endl;
//            truthfile.close();
//            continue;
//        }
//
//        simulator.getNextState();
//        std::vector<Simulator::Dinding*> localMeasurements = simulator.robot->sensedData;
//        SE2 *pose = simulator.robot->simPose;
//        SE2 *truth = simulator.robot->truePose;
//
//        if (localMeasurements.empty())
//            continue;
//
//        std::ofstream truthfile;
//        truthfile.open ("/home/ism/tmp/truth.dat", std::ios::out | std::ios::app);
//        truthfile << truth->toVector().transpose() << std::endl;
//        truthfile.close();
//
//        Pose2::Ptr currentPose = graph2.createPose ();
//        currentPose->setEstimate (*pose);
//
//        PoseMeasurement2::Ptr pm = graph2.createPoseMeasurement();
//
//        pm->vertices()[0] = lastPose.get();
//        pm->vertices()[1] = currentPose.get();
//        pm->setMeasurement (lastPose->estimate().inverse() * *pose);
//        pm->information () = pose_cov.inverse();
//
//        lastPose = currentPose;
//
//        std::vector<Wall2::Ptr> walls = graph2.getWallDB();
//
//        for (int i = 0; i < localMeasurements.size(); i++)
//        {
//            Wall2::Ptr w;
//            bool found = false;
//
//            std::cout << "OBSERVED: WALL WITH ID(s): " << localMeasurements[i]->id << std::endl; 
//
//            for (int j = 0; j < walls.size(); j++)
//            {
//                if (walls[j]->simId == localMeasurements[i]->id)
//                {
//                    std::cout << "WALL IS FOUND IN DATABASE: " << walls[j]->simId << std::endl;
//                    found = true;
//                    w = walls[j];
//                    break;
//                }
//            }
//
//            if (!found)
//            {
//                double rho = localMeasurements[i]->rho;
//                double theta = localMeasurements[i]->theta;
//
//                std::vector<Eigen::Vector2d> points;
//                points.push_back (localMeasurements[i]->p);
//                points.push_back (localMeasurements[i]->q);
//                
//                Eigen::Vector2d global = calc_global (points, currentPose);
//
//                w = graph2.createWall();
//                w->setRho(global[1]);
//                w->setTheta(global[0]);
//                w->simId = localMeasurements[i]->id;
//                graph2.registerWall(w);
//            }
//            
//            WallMeasurement2::Ptr wm = graph2.createWallMeasurement();
//            wm->vertices()[0] = currentPose.get();
//            wm->vertices()[1] = w.get();
//            double measurementData[2] = {localMeasurements[i]->theta, localMeasurements[i]->rho};
//            wm->setMeasurementData (measurementData);
//            wm->information () = wall_cov.inverse();
//        }
//
//
//        if (frame % 3 == 0)
//        {
//            graph2.optimize();
//        }
//
//        frame++;
//        rate.sleep();
//
//        if (frame == 100)
//            break;
//    }
//
//    typedef BlockSolver<BlockSolverTraits<-1,-1> > SlamBlockSolver;
//    typedef LinearSolverCSparse<SlamBlockSolver::PoseLandmarkMatrixType> SlamLinearSolver;
//    SparseOptimizer optimizer;
//    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
//    linearSolver->setBlockOrdering (true);
////    OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton (
////            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
//    OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg (
//            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
//    optimizer.setAlgorithm (solver);
//    optimizer.setVerbose (true);
//
//    Eigen::Matrix<double, 3, 3> posecov;
//    Eigen::Matrix<double, 2, 2> wallcov;
//
//    double _transnoise = 10;
//    double _rotnoise = (0.1 * M_PI/180); 
//    double _wallnoise = 10;
//
//    double _transcov = _transnoise * _transnoise;
//    double _rotcov = _rotnoise * _rotnoise;
//    double _wallcov = _wallnoise * _wallnoise;
//
//    posecov << _transcov, 0, 0,
//               0, _transcov, 0,
//               0, 0, _rotcov;
//
//    wallcov << _wallcov, 0,
//                0, _wallcov;
//
//    SE2 e1 (    3.0, 0.0, 3*M_PI/4);
//    SE2 e2 (    0.0 + gaussian_generator<double>(0.0, _transnoise),
//                4.0 + gaussian_generator<double> (0.0, _transnoise),
//                -M_PI/4 + gaussian_generator<double> (0.0, _rotnoise));
//    SE2 e3 (    1.0 + gaussian_generator<double>(0.0, _transnoise),
//                2.0 + gaussian_generator<double> (0.0, _transnoise),
//                M_PI/4 + gaussian_generator<double> (0.0, _rotnoise));
//    SE2 e4 (    -1.0 + gaussian_generator<double>(0.0, _transnoise),
//                -1.0 + gaussian_generator<double> (0.0, _transnoise),
//                -M_PI + gaussian_generator<double> (0.0, _rotnoise));
//    SE2 e5 (    2.0 + gaussian_generator<double>(0.0, _transnoise),
//                1.0 + gaussian_generator<double> (0.0, _transnoise),
//                M_PI/2 + gaussian_generator<double> (0.0, _rotnoise));
//
//    std::vector<SE2> poses;
//    std::vector<VertexSE2*> posedb;
//    poses.push_back (e1);
//    poses.push_back (e2);
//    poses.push_back (e3);
//    poses.push_back (e4);
//    poses.push_back (e5);
//    
//    for (int i = 0; i < poses.size(); i++)
//    {
//        VertexSE2 *p = new VertexSE2;
//        p->setEstimate(poses[i]);
//        p->setId(i);
//        if (i == 0) p->setFixed (true);
//        posedb.push_back(p);
//        optimizer.addVertex (p);
//
//        if (i > 0)
//        {
//            PoseMeasurement2 *pm = new PoseMeasurement2;
//            pm->vertices()[0] = posedb[i-1];
//            pm->vertices()[1] = posedb[i];
//            pm->setMeasurement (posedb[i-1]->estimate().inverse() * posedb[i]->estimate());
//            pm->information () = posecov;
//            optimizer.addEdge(pm);
//        }
//    }
//
//    Eigen::Vector2d f1 (    sqrt(0.5) + gaussian_generator<double>(0.0,_wallnoise), \
//                            3*M_PI/4 + gaussian_generator<double>(0.0,_rotnoise));
//    Eigen::Vector2d f2 (    1.0 + gaussian_generator<double>(0.0,_wallnoise), \
//                            M_PI/5 + gaussian_generator<double>(0.0,_rotnoise));
//
//    std::vector<Eigen::Vector2d> walls;
//    std::vector<Wall4*> walldb;
//    walls.push_back(f1);
//    walls.push_back(f2);
//
//    int nextId = (*posedb.back()).id();
//    for (int i = 0; i < walls.size(); i++)
//    {
//        Wall4 *w = new Wall4;
//        w->setRho (walls[i][0]);
//        w->setTheta (walls[i][1]);
//        w->setId(++nextId);
//        walldb.push_back(w);
//        optimizer.addVertex(w);
//    }
//
//    for (int i = 0; i < posedb.size(); i++)
//    {
//        for (int j = 0; j < walldb.size(); j++)
//        {
//            double rho = walldb[j]->rho();
//            double theta = walldb[j]->theta();
//            double x = posedb[i]->estimate().toVector()[0];
//            double y = posedb[i]->estimate().toVector()[1];
//            double p = posedb[i]->estimate().toVector()[2];
//            double cosp = cos(p);
//            double sinp = sin(p);
//
//            double m = -1/tan(theta);
//            double c = rho * sqrt(m*m+1);
//            double xintersect = (y-c)/m;
//            double rho_ = rho - x*cosp - y*sinp;
//            double theta_;
//            x < xintersect ? theta_ = theta - p + M_PI : theta_ = theta - p;
//            rho_ += gaussian_generator<double> (0.0, 0.1);
//            theta_ += gaussian_generator<double> (0.0, 5.0*M_PI/180);
//            theta_ = normalize_theta (theta_);
//
//            WallMeasurement4 *wm = new WallMeasurement4;
//            wm->vertices()[0] = posedb[i];
//            wm->vertices()[1] = walldb[j];
//            double data[2] = {theta_, rho_};
//            wm->setMeasurementData (data);
//            wm->information () = wallcov;
//            optimizer.addEdge(wm);
//        }
//    }
//
//    optimizer.save("/home/ism/tmp/before.g2o");
//    optimizer.initializeOptimization();
//    optimizer.optimize(100);
//    optimizer.save("/home/ism/tmp/after.g2o");

	return 0;
}

Eigen::Vector2d calc_global (std::vector<Eigen::Vector2d> points, Pose2::Ptr& pose)
{
    Eigen::Vector3d v = pose->estimate().toVector();
    auto x = v[0];
    auto y = v[1];
    auto phi = v[2];

    double _x1 = points[0].x();
    double _y1 = points[0].y();
    double _x2 = points[1].x();
    double _y2 = points[1].y();

    double cosphi = cos(phi);
    double sinphi = sin(phi);

//    double x1 = _x1 * cosphi + _y1 * sinphi - x * cosphi - y * sinphi;
//    double y1 = -_x1 * sinphi + _y1 * cosphi + x * sinphi - y * cosphi;
//    double x2 = _x2 * cosphi + _y2 * sinphi - x * cosphi - y * sinphi;
//    double y2 = -_x2 * sinphi + _y2 * cosphi + x * sinphi - y * cosphi;

    double x1 = _x1 * cosphi - _y1 * sinphi + x; 
    double y1 = _x1 * sinphi + _y1 * cosphi + y;
    double x2 = _x2 * cosphi - _y2 * sinphi + x; 
    double y2 = _x2 * sinphi + _y2 * cosphi + y;

    double m;
    x2 == x1 ? m = std::numeric_limits<double>::max() : m = (y2-y1)/(x2-x1);
    double c = y1 - m * x1;

    double rho = std::abs(c)/sqrt(m*m+1);
    double cx = -(m*c)/(m*m+1);
    double cy = c/(m*m+1);
    double theta = atan2 (cy,cx);

    Eigen::Vector2d param (theta, rho);
    return param;
}
