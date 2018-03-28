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

#include "pointmatcher/PointMatcher.h"

namespace MYSLAM {
    class System;
    class ParticleFilter;
    class Tracker
    {
        public:
            Tracker(System&);

            Pose::Ptr _lastPose;
            SE2 *_lastOdom;
            SE2 *_measurement;
            pcl::PointCloud<pcl::PointXYZ>::Ptr _lastPCL; 
            sensor_msgs::PointCloud2ConstPtr _lastDP;

            enum {
                 USE_ODOMETRY,
                 USE_ODOMETRY_IMU,
                 USE_CONSTANT_VELOCITY_MODEL,
                 USE_SCAN_MATCHING,
                 USE_PARTICLE_FILTER,
                 USE_POINT_MATCHER,
                 USE_GMAPPING
            };

            int _method;

            void setPrior (Pose::Ptr& p) { _lastPose = p; _lastOdom->fromVector(p->_pose); };
            void setFirstPCL (pcl::PointCloud<pcl::PointXYZ>::Ptr& priorPCL) { _lastPCL = priorPCL; };
            void setFirstDP (const sensor_msgs::PointCloud2ConstPtr& cloudmsg) { _lastDP = cloudmsg; };

            void trackPoseByConstantVelocityModel (double *data, Pose::Ptr& pose);
            void trackPoseByOdometry (double *data, Pose::Ptr& pose);
            void trackPoseByScanMatching (pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl, Pose::Ptr& pose, double *data); 
            void trackPoseByPointMatcher (const sensor_msgs::PointCloud2ConstPtr& cloudmsg, Pose::Ptr& pose, double *data);
            void trackPoseByParticleFilter (double *data, ParticleFilter* pf);
            void run();

        private:
            System *_system;

            tf2_ros::TransformListener *_listener2;
            tf2_ros::Buffer *_buffer;

    };
}
#endif
