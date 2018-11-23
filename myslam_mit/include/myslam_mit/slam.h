#ifndef _SLAM_H_
#define _SLAM_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "myslam_system/myslam_system.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

namespace MYSLAM {
	class Slam {
		public:
			Slam (ros::NodeHandle&);
			void callback (const sensor_msgs::PointCloud2ConstPtr& cloud, 
		                const sensor_msgs::ImageConstPtr& rgb,
		                const sensor_msgs::ImageConstPtr& depth,
				const nav_msgs::OdometryConstPtr& odom,
				const darknet_ros_msgs::BoundingBoxesConstPtr& bb);

			void loadCloudData();
			void loadColoredCloudData();
			void loadRGBData();
			void loadOdomAndObjectData();

			void extractCloud (	const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
						pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud,
						int xmin, int xmax, int ymin, int ymax);
			void extractColoredCloud (	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud,
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outCloud,
						int xmin, int xmax, int ymin, int ymax);

            void removeStructure (  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outCloud); 

            void getObjectPosition (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud, Eigen::Vector3d&, Eigen::Vector3d&);

            void saveData();
            void loadSlamData();
            void run();

		private:
			Graph* _graph;
			Optimizer* _opt;
			ros::NodeHandle* _nh;

			struct frame {
				int index;
                int n_object_detected;
				double x, y, t;
				std::vector<int> xmin;
				std::vector<int> xmax;
				std::vector<int> ymin;
				std::vector<int> ymax;
				std::vector<std::string> category;
				std::vector<int> confidence;
                std::vector<Eigen::Vector3d> pos;
                std::vector<Eigen::Vector3d> boundary;
			};

			std::vector<frame> _frame;
	};
}

#endif
