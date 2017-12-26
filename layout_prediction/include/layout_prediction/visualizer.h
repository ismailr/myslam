#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "layout_prediction/system.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"

namespace MYSLAM {
    class Visualizer
    {
        public:
            Visualizer (ros::NodeHandle, System&, Graph&);
            void visualizeWallOptimizedPq (bool local = false);
            void visualizeWallOptimizedRt (bool local = false);
            void visualizeWallMeasuredPq (Eigen::Vector2d&, Eigen::Vector2d&, bool local = false);
            void visualizeWallMeasuredRt (double, double, bool local = false);
            void visualizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr&);

        private:
            System *_system;
            Graph *_graph;
            ros::NodeHandle _rosnodehandle;
            ros::Publisher _pub_wall_measured_pq;
            ros::Publisher _pub_wall_measured_rt;
            ros::Publisher _pub_wall_optimized_pq;
            ros::Publisher _pub_wall_optimized_rt;
            ros::Publisher _pub_cloud;
    };
}

#endif
