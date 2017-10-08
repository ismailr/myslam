#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include "layout_prediction/tracker.h"
#include "layout_prediction/frame.h"
#include "layout_prediction/pose.h"

bool Tracker::init = true;

Tracker::Tracker (System& system, Graph& graph) 
    :_system (&system), _graph (&graph), _previousFrameProcessed (0) 
{
    _system->setTracker (*this);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr __prevCloud (new pcl::PointCloud<pcl::PointXYZ>);
//    _prevCloud = __prevCloud;
}

void Tracker::run ()
{
    while (1)
    {
        if (_system->_framesQueue.empty())
            continue;

        std::unique_lock <std::mutex> lock_frames_queue (_system->_framesQueueMutex);
        for (int i = 0; i < _system->_framesQueue.size(); ++i)
        {
            Frame::Ptr framePtr (_system->_framesQueue.front());

            if (framePtr->getId() == _previousFrameProcessed) // already process this frame
                continue;

            _previousFrameProcessed = framePtr->getId();

            int useCount = ++framePtr->_useCount; // use the frame and increment count
            track (*framePtr);
            if (useCount == 2) // WallDetector already used it, so pop and delete
            {
                _system->_framesQueue.pop ();
//                delete frame; // it's a must, otherwise memory leak!
            }
        }
        lock_frames_queue.unlock ();

    }
}

void Tracker::track (Frame& frame)
{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = frame.getCloud();
//	pcl::PointCloud<pcl::PointXYZ>::Ptr _laser (new pcl::PointCloud<pcl::PointXYZ>);
//
//	int height = static_cast<int>(cloud->height/4);
//	for(int i = 0; i < cloud->width; i++)
//    {
//        for(int j = cloud->height/2; j < cloud->height; j++)
//        {
//            if (i % 20 == 0 && j % 20 == 0)
//                _laser->push_back(cloud->at(i,j));
//        }
//    }
//
//	_laser->header.frame_id = cloud->header.frame_id;
//	_laser->header.seq = cloud->header.seq;

//    /* NDT */
//    if (Tracker::init)
//    {
//        _prevCloud = _laser;
//        Tracker::init = false;
//        return;
//    }
//
//    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
//    ndt.setTransformationEpsilon (0.01);
//    ndt.setStepSize (0.1);
//    ndt.setResolution (10);
//    ndt.setMaximumIterations (35);
//    ndt.setInputSource (_prevCloud);
//    ndt.setInputTarget (cloud);
//    ndt.align (*_prevCloud);
//
//    Eigen::Matrix4f t = ndt.getFinalTransformation ();
//    _prevCloud = _laser;
//
//    std::cout << t;
//
//    /* END OF NDT */

//    /* ICP */
//    if (Tracker::init)
//    {
//        _prevCloud = _laser;
//        Tracker::init = false;
//        return;
//    }
//
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setMaximumIterations (5);
//    icp.setInputSource (_prevCloud);
//    icp.setInputTarget (_laser);
//    icp.align (*_prevCloud);
//
//    Eigen::Matrix4f t = icp.getFinalTransformation ();
//    _prevCloud = _laser;

//    std::cout << t;

//    /* END OF ICP */

}

Tracker2::Tracker2():_prevTime(0.0){}

void Tracker2::setInitialPose (Pose2& pose)
{
    SE2 t (0.0,0.0,0.0);
    pose.setEstimate (t);
    _lastPose = &pose;
    _prevTime = ros::Time::now().toSec();
}

void Tracker2::estimateFromOdom (const nav_msgs::OdometryConstPtr& odom, Pose2& pose)
{
    tf::Quaternion q (
                odom->pose.pose.orientation.x,
                odom->pose.pose.orientation.y,
                odom->pose.pose.orientation.z,
                odom->pose.pose.orientation.w
            );
    tf::Matrix3x3 m (q);
    double roll, pitch, yaw;
    m.getRPY (roll, pitch, yaw);

    double odom_x = odom->pose.pose.position.x;
    double odom_y = odom->pose.pose.position.y;
    double odom_theta = yaw;
    SE2 t (odom_x, odom_y, odom_theta);
    pose.setEstimate (t);
}

void Tracker2::estimateFromModel (const nav_msgs::OdometryConstPtr& action, Pose2& pose)
{
    double curTime = ros::Time::now().toSec();
    double deltaTime = curTime - _prevTime;

    double vx = action->pose.pose.position.x;
    double vy = action->pose.pose.position.y;
    double w  = action->pose.pose.orientation.z;
    double x0 = _lastPose->estimate()[0];
    double y0 = _lastPose->estimate()[1];
    double theta0 = _lastPose->estimate()[2];

    double x = x0 + (vx * cos(theta0) + vy * sin(theta0)) * deltaTime;
    double y = y0 + (-vx * sin(theta0) + vy * cos(theta0)) * deltaTime;
    double theta = theta0 + w * deltaTime;

    SE2 t (x, y, theta);
    pose.setEstimate (t);

    _prevTime = curTime;
    _lastPose = &pose;
}
