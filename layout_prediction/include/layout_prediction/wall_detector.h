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


class LocalMapper2;
class System;
class System2;
class Graph;
class Graph2;
class Wall;
class Wall2;
class WallDetector
{
    public:
        unsigned long _previousFrameProcessed;
        WallDetector (System&, Graph&);
        void detect (Frame::Ptr&);
        void run ();
        void localToGlobal (Wall::Ptr); // measurement model

    private:
        System *_system;
        Graph *_graph;
        std::vector<line> _lines; // todo: delete!

        void line_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr, int poseId);
        std::vector<int> plane_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr, int poseId);
        geometry_msgs::PointStamped transformPoint (const tf::TransformListener& listener,geometry_msgs::PointStamped p);
};

class WallDetector2
{
    public:
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloudCluster;
        typedef std::vector<Wall2::Ptr> Walls;
        typedef std::vector<WallMeasurement2::Ptr> WallMeasurements;

        WallDetector2(System2&, Graph2&, LocalMapper2&);
        void detect(Pose2::Ptr& pose, const PointCloud::Ptr cloud);

    private:
        System2 *_system;
        Graph2 *_graph;
        LocalMapper2 *_localMapper;

        const int USE_LINE_FITTING = 1;
        const int USE_PLANE_FITTING = 2;
        int _method;

        void prepare_cloud (PointCloud& _preparedCloud, const PointCloud::Ptr cloud);
        void cluster_cloud (PointCloudCluster& cluster, PointCloud& _preparedCloud);
        void line_fitting (std::vector<Eigen::Vector2d>&, PointCloud& _preparedCloud);
        Eigen::Vector2d plane_fitting (PointCloud& _preparedCloud);
        std::vector<Eigen::Vector3d> extract_inliers (pcl::PointIndices::Ptr indices, PointCloud& _preparedCloud);
        void localToGlobal (Wall2::Ptr& wall, Pose2::Ptr& pose);
        Eigen::Vector2d inverse_measurement (Eigen::Vector2d& wParam, Pose2::Ptr& pose);
        Eigen::Vector2d inverse_measurement_from_points (std::vector<Eigen::Vector2d> points, Pose2::Ptr& pose);
};

#endif
