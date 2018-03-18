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

#include <g2o/types/slam2d/se2.h>

#include "layout_prediction/system.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/graph.h"


namespace MYSLAM {
    class Graph;
    class Wall;
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
            void detect2(Pose::Ptr& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
                    std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >& outWalls);
            void prepareCloud (const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud);
            void clusterCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudset);
            void lineFitting (pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, Pose::Ptr& pose, 
                    std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >& outWalls);
            void localToGlobal (Eigen::Vector2d& mc_, Pose::Ptr& pose, Eigen::Vector2d& mc);

            void detectFromMultiplePoses (pcl::PointCloud<pcl::PointXYZ>::Ptr&, std::vector<SE2>&,
                    std::vector<std::vector<Eigen::Vector2d> >& results);
            void lineFittingFromMultiplePoses (pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
                    std::vector<SE2>& poses, 
                    std::vector<std::vector<Eigen::Vector2d> >& results);

            std::vector<Eigen::Vector3d> extractInliers (pcl::PointIndices inliers, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud);

        private:
            int _method;
            System *_system;
    };
}

#endif
