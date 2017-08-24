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

class System;
class WallDetector
{
    public:
        WallDetector ();
        void detect (Frame*);
        void attachTo (System*);
        void run ();

    private:
        System *_system;
        std::vector<line> _lines;

        void line_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<line>&);
        geometry_msgs::PointStamped transformPoint (const tf::TransformListener& listener,geometry_msgs::PointStamped p);
};

#endif
