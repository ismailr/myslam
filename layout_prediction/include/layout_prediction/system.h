#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include "layout_prediction/tracker.h"
#include "layout_prediction/wall_detector.h"

class Tracker;
class WallDetector;
class System
{
    public:
        System(ros::NodeHandle nh);
        void setTracker (Tracker*);
        void setWallDetector (WallDetector*);
        void readSensorsData (const sensor_msgs::PointCloud2ConstPtr&, 
//                const sensor_msgs::ImageConstPtr&,
                const nav_msgs::OdometryConstPtr&);
        void readActionData (const pr2_mechanism_controllers::BaseOdometryState::Ptr&);
        void visualize();
        
    private:
        static unsigned long int _frameId;
        // Main components
        Tracker *_tracker;
        WallDetector *_wall_detector;

        // Ros nodehandle
        ros::NodeHandle _rosnodehandle;

        // Published by system
        ros::Publisher _pub_marker;
        ros::Publisher _pub_cloud;
        ros::Publisher _pub_depth;

};

#endif
