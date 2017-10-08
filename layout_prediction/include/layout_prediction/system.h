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
#include "layout_prediction/tracker.h"
#include "layout_prediction/local_mapper.h"
#include "se2.h"

class WallDetector;
class Optimizer;
class Tracker;
class Frame;
class Graph;
class Pose;
class System
{
    public:
        typedef std::shared_ptr<System> Ptr;
        typedef std::shared_ptr<const System> ConstPtr;

        std::queue <Frame::Ptr> _framesQueue;
        std::mutex _framesQueueMutex;

        System(ros::NodeHandle nh, Graph&);
        void setWallDetector (WallDetector&);
        void setOptimizer (Optimizer&);
        void setTracker (Tracker&);

        void readSensorsData (const sensor_msgs::PointCloud2ConstPtr&, 
                const sensor_msgs::ImageConstPtr&,
                const sensor_msgs::ImageConstPtr&,
                const nav_msgs::OdometryConstPtr&,
                const nav_msgs::OdometryConstPtr&);
        
        template <typename T> void visualize(T&);
        template <typename T> void visualize (std::vector<T>);
        
    private:
        // Main components
        WallDetector *_wall_detector;
        Optimizer *_optimizer;
        Tracker *_tracker;
        Graph *_graph;
        LocalMapper *_localMapper;

        double _previousTime;
        double _currentTime;
        bool _init;

        Pose::Ptr _lastPosePtr;
        Pose::Ptr _currentPosePtr;

        // Ros nodehandle
        ros::NodeHandle _rosnodehandle;

        // Published by system
        ros::Publisher _pub_marker;
        ros::Publisher _pub_cloud;
        ros::Publisher _pub_depth;
        ros::Publisher _pub_rgb;

        SE2* estimateFromOdom (const nav_msgs::OdometryConstPtr&); 
};

class System2
{
    public:
        typedef std::shared_ptr<System> Ptr;
        typedef std::shared_ptr<const System> ConstPtr;

        System2();

        void readSensorsData (
                const sensor_msgs::PointCloud2ConstPtr& cloud, 
                const sensor_msgs::ImageConstPtr& rgb,
                const sensor_msgs::ImageConstPtr& depth,
                const nav_msgs::OdometryConstPtr& odom,
                const nav_msgs::OdometryConstPtr& action);

    private:
        bool _init;
        double _prevTime;
        double _curTime;

        Pose2 _lastPose;
        Pose2 _curPose;

};

#endif
