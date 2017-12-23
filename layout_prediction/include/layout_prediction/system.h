#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "layout_prediction/wall_detector.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/visualizer.h"
#include "se2.h"

class WallDetector2;
class Tracker2;
class Graph;
class Graph2;
class Pose;
class Pose2;
class System2
{
    public:
        static int _framecounter;

        System2(ros::NodeHandle nh, Graph2& graph);
        void set_tracker (Tracker2& tracker) { _tracker = &tracker; };
        void set_wall_detector (WallDetector2& wallDetector) { _wallDetector = &wallDetector; };

        void readSensorsData (
                const sensor_msgs::PointCloud2ConstPtr& cloud, 
                const sensor_msgs::ImageConstPtr& rgb,
                const sensor_msgs::ImageConstPtr& depth,
                const nav_msgs::OdometryConstPtr& odom,
//                const nav_msgs::OdometryConstPtr& action,
                const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined);

        long requestUniqueId () { return _gen.getUniqueId (); };

        template <typename T> void visualize(T&);
        template <typename T> void visualize2(T&);
        template <typename T> void visualize (std::vector<T>);
        void visualize_grad (double m, double c);
        void visualize_rho (double rho, double theta);

        Tracker2* getTracker() const { return _tracker; };

    private:
        bool _init;
        double _prevTime;
        double _curTime;

        ros::NodeHandle _rosnodehandle;
        tf::TransformListener *_listener;
        Graph2 *_graph;
        Tracker2 *_tracker;
        WallDetector2 *_wallDetector;

        IdGenerator _gen;
        std::map<int, std::set<int> > _currentOptimization;

        // Published by system
        ros::Publisher _pub_marker;
        ros::Publisher _pub_marker2;
        ros::Publisher _pub_marker3;
        ros::Publisher _pub_cloud;
        ros::Publisher _pub_depth;
        ros::Publisher _pub_rgb;
        ros::Publisher _pub_odom;
};

namespace MYSLAM {
    class WallDetector;
    class Tracker;
    class Optimizer;
    class Visualizer;
    class System
    {
        public:
            System(ros::NodeHandle nh);

            static unsigned long int _frameCounter;
            double _currentTime;
            double _prevTime;

            // callback
            void readSensorsData (
                const sensor_msgs::PointCloud2ConstPtr& cloud, 
                const sensor_msgs::ImageConstPtr& rgb,
                const sensor_msgs::ImageConstPtr& depth,
                const nav_msgs::OdometryConstPtr& odom,
//                const nav_msgs::OdometryConstPtr& action,
                const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined
                );
        private:
            bool _init;
            ros::NodeHandle _rosnodehandle;
            Graph *_graph;
            Tracker *_tracker;
            WallDetector *_wallDetector;
            Optimizer *_optimizer;
            Visualizer *_visualizer;
            tf::TransformListener *_listener;
    };
}

#endif
