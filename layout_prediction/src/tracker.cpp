#include <iostream>
#include <fstream>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "layout_prediction/tracker.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/settings.h"
#include "layout_prediction/visualizer.h"

Tracker2::Tracker2 (System2& system, Graph2& graph) 
    :_system (&system), _graph (&graph), _prevTime (0.0)
{
    _system->set_tracker (*this);
}

SE2* Tracker2::estimateFromOdom (const OdomConstPtr& odom)
{
    Converter c;
    SE2 *t = new SE2();
    c.odomToSE2 (odom, *t);

    // base_link to base_laser_link
    SE2 *offset = new SE2 (0.275, 0.0, 0.252);
    *t = (*t) * (*offset);

    return t;
}

SE2* Tracker2::estimateFromOdomCombined (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined)
{
    Converter c;
    SE2 *t = new SE2();
    c.odomCombinedToSE2 (odomcombined, *t);

    // base_link to base_laser_link
    SE2 *offset = new SE2 (0.275, 0.0, 0.252);
    *t = (*t) * (*offset);

    return t;
}

SE2* Tracker2::estimateFromModel (const OdomConstPtr& action)
{
    double curTime = ros::Time::now().toSec();
    double deltaTime = curTime - _prevTime;

    double vx = action->pose.pose.position.x;
    double vy = action->pose.pose.position.y;
    double w  = action->pose.pose.orientation.z;
    double x0 = _lastPose->estimate()[0];
    double y0 = _lastPose->estimate()[1];
    double theta0 = _lastPose->estimate()[2];

    double x = x0 + (vx * cos(theta0) - vy * sin(theta0)) * deltaTime;
    double y = y0 + (vx * sin(theta0) + vy * cos(theta0)) * deltaTime;
    double theta = theta0 +  w * deltaTime;


    SE2 *t = new SE2 (x, y, theta);
    SE2 *offset = new SE2 (0.275, 0.0, 0.252);
    *t = (*t) * (*offset);

//    std::ofstream actionfile;
//    actionfile.open ("/home/ism/tmp/action.dat",std::ios::out|std::ios::app);
//    actionfile << t->translation().transpose() << " " << t->rotation().angle() << std::endl;
//    actionfile.close();

    return t;
}

Pose2::Ptr Tracker2::trackPose (const OdomConstPtr& odom, const OdomConstPtr& action, const OdomCombinedConstPtr& odomcombined, bool init) 
{
    double time = ros::Time::now().toSec();

    SE2* t = estimateFromOdom (odom);
//    SE2* t = estimateFromOdomCombined (odomcombined);
    Pose2::Ptr pose = _graph->createPoseWithId();

//    std::ofstream poseb4optfile;

    if (init)
    {
        pose->setEstimate (*t);
        pose->setModel (*t);
        _prevTime = time;
        _lastPose = pose;

        return pose;
    }

    SE2* m = estimateFromModel (action);
    pose->setEstimate (*t);
    pose->setModel (*m);

    Eigen::Matrix<double, 3, 3> inf;
    inf.setIdentity();
//    inf <<  100.0, 0.0, 0.0,
//            0.0, 100.0, 0.0,
//            0.0, 0.0, 5.0;

//    PoseMeasurement2::Ptr pm = _graph->createPoseMeasurement();
    PoseMeasurement2::Ptr pm = _graph->createPoseMeasurement(_lastPose->id(), pose->id());

    pm->vertices()[0] = _lastPose.get();
    pm->vertices()[1] = pose.get();
    pm->setMeasurement (_lastPose->estimate().inverse() * *t);
    pm->information () = inf;
    
    std::ofstream poseb4optfile;
    poseb4optfile.open ("/home/ism/tmp/pose_b4_opt.dat",std::ios::out|std::ios::app);
    poseb4optfile    << pose->estimate().toVector()[0] << " " 
                << pose->estimate().toVector()[1] << " "
                << pose->estimate().toVector()[2] << std::endl;
    poseb4optfile.close();
    
    _prevTime = time;
    _lastPose = pose;

    return pose;
}

namespace MYSLAM {
    Tracker::Tracker(System& system)
        : _system (&system), 
          _method (MYSLAM::TRACKER_METHOD),
          _lastPCL (new pcl::PointCloud<pcl::PointXYZ>)
    {
        _buffer = new tf2_ros::Buffer;
        _listener2 = new tf2_ros::TransformListener (*_buffer);
        _lastOdom = new SE2();
//        _lastPCL = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    };

//    SE2* Tracker::odomToSE2 (const nav_msgs::OdometryConstPtr& odom)
//    {
////        geometry_msgs::TransformStamped transformStamped;
////        try {
////            transformStamped = _buffer->lookupTransform (
////                    "base_link",
////                    "base_laser_link",
////                    ros::Time(0));
////
////        } catch (tf2::TransformException &ex) {
////            ROS_WARN ("%s", ex.what());
////            ros::Duration(1.0).sleep();
////        }
////
////        double x = transformStamped.transform.translation.x;
////        double y = transformStamped.transform.translation.y;
////        double z = transformStamped.transform.translation.z;
////        double qx = transformStamped.transform.rotation.x;
////        double qy = transformStamped.transform.rotation.y;
////        double qz = transformStamped.transform.rotation.z;
////        double qw = transformStamped.transform.rotation.w;
////
////        tf2::Quaternion q (qx, qy, qz, qw);
////        double angle = tf2::impl::getYaw(q);
////
////        SE2 transform (x, y, angle);
////        SE2 odomSE2;
////
////        Converter c;
////        c.odomToSE2 (odom, odomSE2);
////        SE2 *laserPose = new SE2;
////        *laserPose = odomSE2 * transform;
////        return laserPose;
//
//        Converter c;
//        SE2 *t = new SE2();
//        c.odomToSE2 (odom, *t);
//        return t;
//    }

    void Tracker::trackPoseByOdometry (double* d, Pose::Ptr& pose)
    {
        // heading change
        double dt = d[2] - _lastOdom->rotation().angle();

        // distance
        double x1 = d[0];
        double y1 = d[1];
        double x0 = _lastOdom->translation().x();
        double y0 = _lastOdom->translation().y();
        double dr = sqrt ((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));

        // handling backward motion
        double current_heading = normalize_angle (_lastPose->_pose[2] + dt);
        if (current_heading < M_PI/2 || current_heading > 3*M_PI/2)
            x1 > x0 ? dr = dr : dr = -dr;
        else if (current_heading > M_PI/2 && current_heading < 3*M_PI/2)
            x1 > x0 ? dr = -dr : dr = dr;
        else if (current_heading == M_PI/2)
            y1 > y0 ? dr = dr : dr = -dr;
        else if (current_heading == 3*M_PI/2)
            y1 > y0 ? dr = -dr : dr = dr;

        double dx = dr * cos (current_heading);
        double dy = dr * sin (current_heading);
        Eigen::Vector3d incr (dx, dy, dt);

        pose->_pose = _lastPose->_pose + incr;
        _lastPose = pose;
        _lastOdom->fromVector (Eigen::Vector3d (d[0], d[1], d[2]));

        ofstream odomfile;
        odomfile.open ("/home/ism/tmp/odom.dat", std::ios::out | std::ios::app);
        odomfile << pose->_pose[0] << " " << pose->_pose[1] << " " << pose->_pose[2] << std::endl;
        odomfile.close();
    }

    void Tracker::trackPoseByConstantVelocityModel (double *data, Pose::Ptr& pose) 
    {
        double deltaTime = _system->_currentTime - _system->_prevTime;

        double vx = data[0];
        double vy = data[1];
        double w  = data[2];
        double x0 = _lastPose->_pose[0];
        double y0 = _lastPose->_pose[1];
        double t0 = _lastPose->_pose[2];

        // motion model
        double x = x0 + (vx * cos(t0) - vy * sin(t0)) * deltaTime;
        double y = y0 + (vx * sin(t0) + vy * cos(t0)) * deltaTime;
        double t = t0 + w * deltaTime;

        pose->_pose << x, y, t;
        _lastPose = pose;

        ofstream actfile;
        actfile.open ("/home/ism/tmp/action.dat", std::ios::out | std::ios::app);
        actfile << pose->_pose[0] << " " << pose->_pose[1] << " " << pose->_pose[2] << std::endl;
        actfile.close();
    }

    void Tracker::trackPoseByScanMatching (pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl, Pose::Ptr& pose, double *d) 
    {
        // heading change
        double dt = d[2] - _lastOdom->rotation().angle();

        // distance
        double x1 = d[0];
        double y1 = d[1];
        double x0 = _lastOdom->translation().x();
        double y0 = _lastOdom->translation().y();
        double dr = sqrt ((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));

        // handling backward motion
        double current_heading = normalize_angle (_lastPose->_pose[2] + dt);
        if (current_heading < M_PI/2 || current_heading > 3*M_PI/2)
            x1 > x0 ? dr = dr : dr = -dr;
        else if (current_heading > M_PI/2 && current_heading < 3*M_PI/2)
            x1 > x0 ? dr = -dr : dr = dr;
        else if (current_heading == M_PI/2)
            y1 > y0 ? dr = dr : dr = -dr;
        else if (current_heading == 3*M_PI/2)
            y1 > y0 ? dr = -dr : dr = dr;

        double dx = dr * cos (current_heading);
        double dy = dr * sin (current_heading);

//
//        double deltaTime = _system->_currentTime - _system->_prevTime;
//
//        double vx = d[0];
//        double vy = d[1];
//        double w  = d[2];
//        double x0 = _lastPose->_pose[0];
//        double y0 = _lastPose->_pose[1];
//        double t0 = _lastPose->_pose[2];
//
//        // motion model
//        double dx = (vx * cos(t0) - vy * sin(t0)) * deltaTime;
//        double dy = (vx * sin(t0) + vy * cos(t0)) * deltaTime;
//        double dt = w * deltaTime;
//
//        pose->_pose << x0 + dx, y0 + dy, t0 + dt;
//        _lastPose = pose;

        pcl::PointCloud<pcl::PointXYZ>::Ptr _f_lastPCL (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize (0.2, 0.2, 0.2);
        filter.setInputCloud (_lastPCL);
        filter.filter (*_f_lastPCL);

        pcl::NormalDistributionsTransform <pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setTransformationEpsilon (0.01);
        ndt.setStepSize (0.1);
        ndt.setResolution (1.0);
        ndt.setMaximumIterations (35);
        ndt.setInputSource (_f_lastPCL);
        ndt.setInputTarget (pcl);

        Eigen::AngleAxisf init_rotation (dt, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f init_translation (dx, dy, 0);
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

        pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align (*outcloud, init_guess);

        _system->getVisualizer()->visualizeCloud (outcloud);

        Eigen::Matrix4f tr = ndt.getFinalTransformation();
        double tx = tr(0,3);
        double ty = tr(1,3);
        double tt = acos(tr(0,1));

        Eigen::Vector3d incr (tx, ty, tt);
        pose->_pose = _lastPose->_pose + incr;
        _lastPose = pose;
        _lastOdom->fromVector (Eigen::Vector3d (d[0], d[1], d[2]));

        ofstream myfile;
        myfile.open ("/home/ism/tmp/ndt.txt", std::ios::out | std::ios::app);
        myfile << "TRAN " << tx << " " << ty << " " << tt << std::endl;
        myfile << "ODOM " << dx << " " << dy << " " << dt << std::endl;
        myfile << std::endl;
        myfile.close();
    }

    void Tracker::trackPoseByParticleFilter (double *data, ParticleFilter* pf) {
        double deltaTime = _system->_currentTime - _system->_prevTime;

        double vx = data[0];
        double vy = data[1];
        double w  = data[2];

        for (int i = 0; i < pf->N; i++) {

            double x0 = pf->_particles[i].pose.translation().x();
            double y0 = pf->_particles[i].pose.translation().y();
            double t0 = pf->_particles[i].pose.rotation().angle();

            // motion model
            double x = x0 + (vx * cos(t0) - vy * sin(t0)) * deltaTime;
            double y = y0 + (vx * sin(t0) + vy * cos(t0)) * deltaTime;
            double t = t0 + w * deltaTime;

            double xnoise = gaussian_generator<double>(0.0, _system->_sigX);
            double ynoise = gaussian_generator<double>(0.0, _system->_sigY);
            double tnoise = gaussian_generator<double>(0.0, _system->_sigT);

            pf->_particles[i].pose.setTranslation (Eigen::Vector2d (x + xnoise, y + ynoise));
            pf->_particles[i].pose.setRotation (Eigen::Rotation2Dd (t + tnoise));

            std::cout << pf->_particles[i].pose.toVector().transpose() << std::endl;
        }
    }
}
