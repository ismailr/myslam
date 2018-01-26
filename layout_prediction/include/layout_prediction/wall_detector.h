#ifndef _WALL_DETECTOR_H_
#define _WALL_DETECTOR_H_

#include <queue>
#include <mutex>
#include <set>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include "layout_prediction/system.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/graph.h"


class System2;
class Graph;
class Graph2;
class Wall;
class Wall2;
class WallDetector2
{
    public:
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloudCluster;
        typedef std::vector<Wall2::Ptr> Walls;
        typedef std::vector<WallMeasurement2::Ptr> WallMeasurements;

        WallDetector2(System2&, Graph2&);
        void detect(Pose2::Ptr& pose, const PointCloud::Ptr cloud, std::set<int>&);

    private:
        System2 *_system;
        Graph2 *_graph;

        const int USE_LINE_FITTING = 1;
        const int USE_PLANE_FITTING = 2;
        int _method;

        void prepare_cloud (PointCloud& _preparedCloud, const PointCloud::Ptr cloud);
        void cluster_cloud (PointCloudCluster& cluster, PointCloud& _preparedCloud);
        void line_fitting (std::vector<Eigen::Vector2d>&, PointCloud& _preparedCloud);
        void line_fitting (PointCloud& _preparedCloud, Pose2::Ptr& pose, std::set<int>& walls);
        Eigen::Vector2d plane_fitting (PointCloud& _preparedCloud);
        std::vector<Eigen::Vector3d> extract_inliers (pcl::PointIndices indices, PointCloud& _preparedCloud);
        void localToGlobal (Wall2::Ptr& wall, Pose2::Ptr& pose);
        void localToGlobal (double* mc_, Pose2::Ptr& pose, double* mc);
        Eigen::Vector2d inverse_measurement (Eigen::Vector2d& wParam, Pose2::Ptr& pose);
        Eigen::Vector2d inverse_measurement_from_points (std::vector<Eigen::Vector2d> points, Pose2::Ptr& pose);
};

namespace MYSLAM {
    class System;
    class WallDetector
    {
        public:
            enum method {
                LINE_FITTING = 0,
                PLANE_FITTING = 1
            };

        public:
            WallDetector(System&);

            void detect(Pose::Ptr& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
                    std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >& outWalls);
            void prepareCloud (const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud);
            void clusterCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudset);
            void lineFitting (pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, Pose::Ptr& pose, 
                    std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >& outWalls);
            void localToGlobal (Eigen::Vector2d& mc_, Pose::Ptr& pose, Eigen::Vector2d& mc);

            std::vector<Eigen::Vector3d> extractInliers (pcl::PointIndices inliers, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud);

        private:
            int _method;
            System *_system;
    };
}

#endif
