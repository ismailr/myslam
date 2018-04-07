#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>

#include "layout_prediction/wall_detector.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/visualizer.h"
#include "layout_prediction/particle_filter.h"
#include "layout_prediction/ekf_mapper.h"
#include <g2o/types/slam2d/se2.h>

namespace MYSLAM {
    class System
    {
        public:
            System(ros::NodeHandle nh);
            Tracker* getTracker() const { return _tracker;};

            enum {BA, PF, EKF}; // Bundle Adjustment, Particle Filter, Extended Kalman Filter
            int _method;

            static unsigned long int _frameCounter;
            double _currentTime;
            double _prevTime;

            const double _sigX = 1e-2;
            const double _sigV = 0.3;
            const double _sigT = 3.0 * M_PI/180.0;
            const double _sigW = 1e-2;
            Eigen::Matrix3d _Q;  // motion noise covariance                  
            Eigen::Matrix2d _R;  // measurement noise covariance

            // callback
            void readSensorsData (
                const sensor_msgs::PointCloud2ConstPtr& cloud, 
//                const sensor_msgs::ImageConstPtr& rgb,
//                const sensor_msgs::ImageConstPtr& depth,
                const nav_msgs::OdometryConstPtr& odom//,
//                const nav_msgs::OdometryConstPtr& action,
//                const nav_msgs::OdometryConstPtr& wodom,
//                const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined
                );

            Visualizer* getVisualizer () { return _visualizer; };

        private:
            bool _init;
            ros::NodeHandle _rosnodehandle;
            Graph *_graph;
            Tracker *_tracker;
            WallDetector *_wallDetector;
            Optimizer *_optimizer;
            Visualizer *_visualizer;
            tf::TransformListener *_listener;
            tf2_ros::TransformListener *_listener2;
            tf2_ros::Buffer *_buffer;
            ParticleFilter* _pf;
            EKFMapper* _ekfm;
    };
}

#endif
