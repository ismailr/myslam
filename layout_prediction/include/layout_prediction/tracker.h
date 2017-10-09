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
        Tracker2(System2&, Graph2&);
        typedef nav_msgs::OdometryConstPtr OdomConstPtr;
        long trackPose (const OdomConstPtr& odom, const OdomConstPtr& action, bool init = false);

    private:
        System2 *_system;
        Graph2 *_graph;
        Pose2* _lastPose;
        double _prevTime;

        void estimateFromOdom (const OdomConstPtr& odom, Pose2& pose);
        void estimateFromModel (const OdomConstPtr& action, Pose2& pose);
};

#endif
