#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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

Eigen::Vector2d calc_global (std::vector<Eigen::Vector2d> points, Pose2::Ptr& pose);

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

    typedef message_filters::sync_policies::ApproximateTime
        <   sensor_msgs::PointCloud2, 
            sensor_msgs::Image, 
            sensor_msgs::Image, 
//            nav_msgs::Odometry, 
            nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10),    cloud_sub, 
                                                                            rgb_sub, 
                                                                            depth_sub, 
                                                                            odometry_sub//, 
                                                                            /*action_sub*/);

    sync.registerCallback (boost::bind (&System2::readSensorsData, &system2, _1, _2, _3, _4/*, _5*/));

//    ros::spin();
//    graph2.optimize();    
//    ros::Rate rate(30.0);
//    bool init = true;
//    Pose2::Ptr lastPose; 
//    int frame = 0;
//
    Eigen::Matrix<double, 3, 3> pose_cov;
    Eigen::Matrix<double, 2, 2> wall_cov;

    double xcov = 0.05 * 0.05;
    double ycov = 0.05 * 0.05;
    double pcov = (0.1 * M_PI/180) * (0.1 * M_PI/180);
    double rcov = (0.5 * 0.5);
    double tcov = (0.1 * M_PI/180) * (0.1 * M_PI/180);

    pose_cov << xcov, 0, 0,
                0, ycov, 0,
                0, 0, pcov;

    wall_cov << rcov, 0,
                0, tcov;
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
////        if (frame % 3 == 0)
////        {
////            graph2.optimize();
////        }
//
//        frame++;
//        rate.sleep();
//
//        if (frame == 100)
//            break;
//    }

    Pose2::Ptr p1 = graph2.createPose();
    Pose2::Ptr p2 = graph2.createPose();
    SE2 e1 (3.7, -5.2, 100*M_PI/180.0);
    SE2 e2 (5.1, 1.9, -40*M_PI/180.0);
    p1->setEstimate(e1);
    p2->setEstimate(e2);

    Wall2::Ptr w = graph2.createWall();
    w->setRho (2.71); //2.68
    w->setTheta (2.57); //2.68
    graph2.registerWall(w);

    double rho1, theta1, rho2, theta2;
    rho1 = std::abs (w->rho() - p1->estimate().translation().x()*cos(w->theta()) - p1->estimate().translation().y()*sin(w->theta()));
    rho2 = std::abs (w->rho() - p2->estimate().translation().x()*cos(w->theta()) - p2->estimate().translation().y()*sin(w->theta()));
    theta1 = w->theta() - p1->estimate().rotation().angle();
    theta2 = w->theta() - p2->estimate().rotation().angle();

    PoseMeasurement2::Ptr pm = graph2.createPoseMeasurement();
    pm->vertices()[0] = p1.get();
    pm->vertices()[1] = p2.get();
    pm->setMeasurement (p1->estimate().inverse() * p2->estimate());
    Eigen::Matrix<double, 3, 3> inf;
    inf.setIdentity();
    pm->information () = pose_cov.inverse();

    Eigen::Matrix<double, 2, 2> inf2;
    inf2.setIdentity();
    WallMeasurement2::Ptr wm1 = graph2.createWallMeasurement();
    wm1->vertices()[0] = p1.get();
    wm1->vertices()[1] = w.get();
    double data1[2] = {theta1, rho1};
    wm1->setMeasurementData (data1);
    wm1->information () = wall_cov.inverse();

    Eigen::Matrix<double, 2, 2> inf3;
    inf3.setIdentity();
    WallMeasurement2::Ptr wm2 = graph2.createWallMeasurement();
    wm2->vertices()[0] = p2.get();
    wm2->vertices()[1] = w.get();
    double data2[2] = {theta2, rho2};
    wm2->setMeasurementData (data2);
    wm2->information () = wall_cov.inverse();

    graph2.optimize();

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
