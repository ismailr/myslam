#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include "layout_prediction/system.h"
#include "layout_prediction/pose.h"

class System2;
class Graph2;

class Tracker2
{
    public:
        Tracker2(System2&, Graph2&);
        typedef nav_msgs::OdometryConstPtr OdomConstPtr;
        typedef geometry_msgs::PoseWithCovarianceStampedConstPtr OdomCombinedConstPtr;
        Pose2::Ptr trackPose (const OdomConstPtr& odom, const OdomConstPtr& action, const OdomCombinedConstPtr& odomcombined, bool init = false);

        void fixLastPose (Pose2::Ptr p) { _lastPose = p; };

    private:
        System2 *_system;
        Graph2 *_graph;
        Pose2::Ptr _lastPose;
        double _prevTime;

        SE2* estimateFromOdom (const OdomConstPtr& odom);
        SE2* estimateFromOdomCombined (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombine);
        SE2* estimateFromModel (const OdomConstPtr& action);
};

namespace MYSLAM {
    class System;
    class Tracker
    {
        public:
            Tracker(System&);

            Pose::Ptr _lastPose;
            SE2 *_lastOdom;

            Pose::Ptr trackPose (
                    const nav_msgs::OdometryConstPtr& odom, 
                    const nav_msgs::OdometryConstPtr& action, 
                    const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined, 
                    bool init = false);

        private:
            System *_system;
            SE2* odomToSE2 (const nav_msgs::OdometryConstPtr& odom);
            SE2* estimateFromOdomCombined (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombine);
            SE2* actionToSE2 (const nav_msgs::OdometryConstPtr& action);

    };
}

#endif
