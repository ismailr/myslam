#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "myslam_system/graph.h"

namespace MYSLAM {
    class Visualizer
    {
        public:
            static int marker_id;
            Visualizer (ros::NodeHandle, Graph&);
            void visualizeWallOptimizedPq (bool local = false);
            void visualizeWallOptimizedRt (bool local = false);
            void visualizeWallMeasuredPq (Eigen::Vector2d&, Eigen::Vector2d&, bool local = false);
            void visualizeWallMeasuredRt (double, double, bool local = false);
            void visualizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	    void visualizeObjects ();
	    void visualizeObjects (std::vector<int>& objects); 
	    void visualizeObjectsGT (const gazebo_msgs::ModelStates::ConstPtr& o);

        private:
            Graph *_graph;
            ros::NodeHandle _rosnodehandle;
            ros::Publisher _pub_wall_measured_pq;
            ros::Publisher _pub_wall_measured_rt;
            ros::Publisher _pub_wall_optimized_pq;
            ros::Publisher _pub_wall_optimized_rt;
            ros::Publisher _pub_cloud;
            ros::Publisher _pub_objects;
            ros::Publisher _pub_objects_gt;
            ros::Publisher _pub_objects_id;
    };
}

#endif
