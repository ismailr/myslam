#include <iostream>
#include <fstream>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include "layout_prediction/tracker.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/settings.h"

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
    Tracker::Tracker(System& system): _system (&system) {
        _buffer = new tf2_ros::Buffer;
        _listener2 = new tf2_ros::TransformListener (*_buffer);
    };

    SE2* Tracker::odomToSE2 (const nav_msgs::OdometryConstPtr& odom)
    {
//        geometry_msgs::TransformStamped transformStamped;
//        try {
//            transformStamped = _buffer->lookupTransform (
//                    "base_link",
//                    "base_laser_link",
//                    ros::Time(0));
//
//        } catch (tf2::TransformException &ex) {
//            ROS_WARN ("%s", ex.what());
//            ros::Duration(1.0).sleep();
//        }
//
//        double x = transformStamped.transform.translation.x;
//        double y = transformStamped.transform.translation.y;
//        double z = transformStamped.transform.translation.z;
//        double qx = transformStamped.transform.rotation.x;
//        double qy = transformStamped.transform.rotation.y;
//        double qz = transformStamped.transform.rotation.z;
//        double qw = transformStamped.transform.rotation.w;
//
//        tf2::Quaternion q (qx, qy, qz, qw);
//        double angle = tf2::impl::getYaw(q);
//
//        SE2 transform (x, y, angle);
//        SE2 odomSE2;
//
//        Converter c;
//        c.odomToSE2 (odom, odomSE2);
//        SE2 *laserPose = new SE2;
//        *laserPose = odomSE2 * transform;
//        return laserPose;

        Converter c;
        SE2 *t = new SE2();
        c.odomToSE2 (odom, *t);
        return t;
    }

    SE2* Tracker::estimateFromOdomCombined (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined)
    {
        Converter c;
        SE2 *t = new SE2();
        c.odomCombinedToSE2 (odomcombined, *t);
        return t;
    }

    SE2* Tracker::actionToSE2 (const nav_msgs::OdometryConstPtr& action)
    {
        double deltaTime = _system->_currentTime - _system->_prevTime;

        double vx = action->pose.pose.position.x;
        double vy = action->pose.pose.position.y;
        double w  = action->pose.pose.orientation.z;
//        double x0 = _lastPose->_pose[0];
//        double y0 = _lastPose->_pose[1];
//        double theta0 = _lastPose->_pose[2];
        double x0 = _lastAction->translation().x();
        double y0 = _lastAction->translation().y();
        double theta0 = _lastAction->rotation().angle();

        double x = x0 + (vx * cos(theta0) - vy * sin(theta0)) * deltaTime;
        double y = y0 + (vx * sin(theta0) + vy * cos(theta0)) * deltaTime;
        double theta = theta0 +  w * deltaTime;

        SE2 *t = new SE2 (x, y, theta);
        return t;
    }

    Pose::Ptr Tracker::trackPose (
            const nav_msgs::OdometryConstPtr& odom, 
            const nav_msgs::OdometryConstPtr& action, 
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined, 
            bool init) 
    {
        SE2* m = odomToSE2 (odom);
//        SE2* t = estimateFromOdomCombined (odomcombined);
        Pose::Ptr pose (new Pose); 

        if (init)
        {
            pose->_pose = m->toVector();
            pose->_poseByModel = pose->_pose;
            pose->_measurement = Eigen::Vector3d();
            pose->_previousId = -1;
            _lastPose = pose;
            _lastOdom = m;
            _lastAction = m;
            return pose;
        }

        SE2* t = actionToSE2 (action);

        // heading change
        double dt = t->rotation().angle() - _lastAction->rotation().angle();

        // distance
        double x1 = t->translation().x();
        double y1 = t->translation().y();
        double x0 = _lastAction->translation().x();
        double y0 = _lastAction->translation().y();
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
        pose->_poseByModel = m->toVector();
        pose->_measurement = incr;
        pose->_previousId = _lastPose->_id;
        _lastPose = pose;
        _lastOdom = m;
        _lastAction = t;

        ofstream actfile;
        actfile.open ("/home/ism/tmp/action.dat", std::ios::out | std::ios::app);
        actfile << t->translation().x() << " " << t->translation().y() << " " << t->rotation().angle() << std::endl;
        actfile.close();

        ofstream odomfile;
        odomfile.open ("/home/ism/tmp/odom.dat", std::ios::out | std::ios::app);
        odomfile << m->translation().x() << " " << m->translation().y() << " " << m->rotation().angle() << std::endl;
        odomfile.close();

        return pose;
    }

    Pose::Ptr Tracker::trackPose (
            SE2 odom,
            const nav_msgs::OdometryConstPtr& action, 
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined, 
            bool init) 
    {
        SE2 *t = new SE2 (odom);
        Pose::Ptr pose (new Pose); 
        
        if (init)
        {
            pose->_pose = t->toVector();
            pose->_poseByModel = pose->_pose;
            _lastPose = pose;
            _lastOdom = t;
            return pose;
        }

        SE2* m = actionToSE2 (action);

        // heading change
        double dt = normalize_theta (t->rotation().angle() - _lastOdom->rotation().angle());

        // distance
        double x1 = t->translation().x();
        double y1 = t->translation().y();
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
        pose->_poseByModel = m->toVector();
        _lastPose = pose;
        _lastOdom = t;

        ofstream actfile;
        actfile.open ("/home/ism/tmp/action.dat", std::ios::out | std::ios::app);
        actfile << m->translation().x() << " " << m->translation().y() << " " << m->rotation().angle() << std::endl;
        actfile.close();

        ofstream odomfile;
        odomfile.open ("/home/ism/tmp/odom.dat", std::ios::out | std::ios::app);
        odomfile << t->translation().x() << " " << t->translation().y() << " " << t->rotation().angle() << std::endl;
        odomfile.close();

        return pose;
    }

}
