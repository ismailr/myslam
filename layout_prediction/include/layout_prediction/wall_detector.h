#ifndef _WALL_DETECTOR_H_
#define _WALL_DETECTOR_H_

#include <queue>
#include <mutex>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "layout_prediction/system.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/frame.h"
#include "layout_prediction/graph.h"

class System;
class Graph;
class Wall;
class WallDetector
{
    public:
        unsigned long _previousFrameProcessed;
        WallDetector (System&, Graph&);
        void detect (Frame::Ptr&);
        void run ();
        void associate_walls (std::vector<Wall*>);

    private:
        System *_system;
        Graph *_graph;
        std::vector<line> _lines; // todo: delete!

        void line_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr, Pose::Ptr&);
        std::vector<Wall*> plane_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr, Pose::Ptr&);
        geometry_msgs::PointStamped transformPoint (const tf::TransformListener& listener,geometry_msgs::PointStamped p);
};

#endif
