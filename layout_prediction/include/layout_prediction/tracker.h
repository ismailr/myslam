#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include "layout_prediction/system.h"
#include "layout_prediction/frame.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/local_mapper.h"

class System;
class Graph;
class System2;
class Graph2;
class Frame;
class Tracker
{
    public:
        static bool init;
        unsigned long _previousFrameProcessed;

        Tracker (System&, Graph&);
        void run ();
        void track (Frame&);

    private:
        System *_system;
        Graph *_graph;

        pcl::PointCloud<pcl::PointXYZ>::Ptr _prevCloud; 
};

class Tracker2
{
    public:
        Tracker2(System2&, Graph2&, LocalMapper2&);
        typedef nav_msgs::OdometryConstPtr OdomConstPtr;
        typedef geometry_msgs::PoseWithCovarianceStampedConstPtr OdomCombinedConstPtr;
        Pose2::Ptr trackPose (const OdomConstPtr& odom, const OdomConstPtr& action, const OdomCombinedConstPtr& odomcombined, bool init = false);

    private:
        System2 *_system;
        Graph2 *_graph;
        LocalMapper2 *_localMapper;
        Pose2::Ptr _lastPose;
        double _prevTime;

        SE2* estimateFromOdom (const OdomConstPtr& odom);
        SE2* estimateFromOdomCombined (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombine);
        SE2* estimateFromModel (const OdomConstPtr& action);
};

#endif
