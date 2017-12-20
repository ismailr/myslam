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
    Tracker::Tracker(System& system): _system (&system) {};

    SE2* Tracker::estimateFromOdom (const nav_msgs::OdometryConstPtr& odom)
    {
        Converter c;
        SE2 *t = new SE2();
        c.odomToSE2 (odom, *t);

        // base_link to base_laser_link
        SE2 *offset = new SE2 (0.275, 0.0, 0.252);
        *t = (*t) * (*offset);

        return t;
    }

    SE2* Tracker::estimateFromOdomCombined (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined)
    {
        Converter c;
        SE2 *t = new SE2();
        c.odomCombinedToSE2 (odomcombined, *t);

        // base_link to base_laser_link
        SE2 *offset = new SE2 (0.275, 0.0, 0.252);
        *t = (*t) * (*offset);

        return t;
    }

    SE2* Tracker::estimateFromModel (const nav_msgs::OdometryConstPtr& action)
    {
        double deltaTime = _system->_currentTime - _system->_prevTime;

        double vx = action->pose.pose.position.x;
        double vy = action->pose.pose.position.y;
        double w  = action->pose.pose.orientation.z;
        double x0 = _lastPose->_pose[0];
        double y0 = _lastPose->_pose[1];
        double theta0 = _lastPose->_pose[2];

        double x = x0 + (vx * cos(theta0) - vy * sin(theta0)) * deltaTime;
        double y = y0 + (vx * sin(theta0) + vy * cos(theta0)) * deltaTime;
        double theta = theta0 +  w * deltaTime;

        SE2 *t = new SE2 (x, y, theta);
        SE2 *offset = new SE2 (0.275, 0.0, 0.252);
        *t = (*t) * (*offset);

        return t;
    }

    Pose::Ptr Tracker::trackPose (
            const nav_msgs::OdometryConstPtr& odom, 
            const nav_msgs::OdometryConstPtr& action, 
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined, 
            bool init) 
    {
        SE2* t = estimateFromOdom (odom);
    //    SE2* t = estimateFromOdomCombined (odomcombined);
        Pose::Ptr pose (new Pose); 

        if (init)
        {
            pose->_pose = t->toVector();
            pose->_poseByModel = pose->_pose;
            _lastPose = pose;
            return pose;
        }

        SE2* m = estimateFromModel (action);
        pose->_pose = t->toVector();
        pose->_poseByModel = m->toVector();
        _lastPose = pose;

        return pose;
    }

}
