#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/impl/utils.h>

#include "layout_prediction/system.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/particle.h"
#include "layout_prediction/particle_filter.h"

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
            pcl::PointCloud<pcl::PointXYZ>::Ptr _lastPCL; 

            enum {
                 USE_ODOMETRY;
                 USE_ODOMETRY_IMU;
                 USE_CONSTANT_VELOCITY_MODEL;
                 USE_SCAN_MATCHING;
                 USE_PARTICLE_FILTER;
            };

            int _method;

            void setPrior (Pose::Ptr& p) { _lastPose = p; _lastOdom->fromVector(p->_pose); };
            void setFirstPCL (pcl::PointCloud<pcl::PointXYZ>::Ptr& priorPCL) { _lastPCL = priorPCL; };

            void trackPoseByConstantVelocityModel (double *data, Pose::Ptr& pose);
            void trackPoseByOdometry (double *data, Pose::Ptr& pose);
            void trackPoseByScanMatching (pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl, Pose::Ptr& pose, double *data); 
            void trackPoseByParticleFilter (double *data, ParticleFilter* pf);

        private:
            System *_system;

            tf2_ros::TransformListener *_listener2;
            tf2_ros::Buffer *_buffer;

    };
}
#endif
