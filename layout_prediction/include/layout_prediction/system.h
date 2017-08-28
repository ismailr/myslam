#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include "layout_prediction/wall_detector.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/frame.h"

class WallDetector;
class Optimizer;
class Frame;
class Graph;
class System
{
    public:
        Graph *_graph;
        std::queue <Frame*> _framesQueue;
        std::mutex _framesQueueMutex;

        System(ros::NodeHandle nh);
        void setWallDetector (WallDetector&);
        void setOptimizer (Optimizer&);

        void readSensorsData (const sensor_msgs::PointCloud2ConstPtr&, 
                const sensor_msgs::ImageConstPtr&,
                const sensor_msgs::ImageConstPtr&,
                const nav_msgs::OdometryConstPtr&);
        void readActionData (const pr2_mechanism_controllers::BaseOdometryState::Ptr&);
        
        std::queue <Frame*> getFramesQueue ();

        template <typename T>
        void visualize(T&);
        
    private:
        // Main components
        WallDetector *_wall_detector;
        Optimizer *_optimizer;

        // Ros nodehandle
        ros::NodeHandle _rosnodehandle;

        // Published by system
        ros::Publisher _pub_marker;
        ros::Publisher _pub_cloud;
        ros::Publisher _pub_depth;
        ros::Publisher _pub_rgb;
};

#endif
