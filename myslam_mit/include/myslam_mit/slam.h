#ifndef _SLAM_H_
#define _SLAM_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

#include "myslam_system/myslam_system.h"
#include "myslam_system/data_association.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

#include <queue>
#include <mutex>

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
            void loadSlamData();

			void extractCloud (	const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
						pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud,
						int xmin, int xmax, int ymin, int ymax);
			void extractColoredCloud (	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud,
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outCloud,
						int xmin, int xmax, int ymin, int ymax);

            void removeStructure (  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outCloud); 

            void getObjectPosition (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud, Eigen::Vector3d&, Eigen::Vector3d&);
            g2o::Vector3 getObjectPosition (pcl::PointCloud<pcl::PointXYZ> cloud, darknet_ros_msgs::BoundingBox bb); 

            void run();
            void init() { _frameToGraphMap[0] = frameToGraph(0);};
            int frameToGraph(int frame_id); // return id of the node in the graph
            g2o::Isometry3 calculateOdometryIncrement (int from, int to);

			void saveData (const sensor_msgs::PointCloud2ConstPtr& cloud, 
				const nav_msgs::OdometryConstPtr& odom,
				const darknet_ros_msgs::BoundingBoxesConstPtr& bb);

            void test();
            void thread1(); 
            void thread2(); 

            g2o::SE3Quat odomToSE3Quat(nav_msgs::Odometry o); 

            g2o::Isometry3 _lastOdom;
            int _lastPoseId;
            static bool _init;
            static int _frameNum;

            std::map<int, std::string> objectClass;
            int getObjectClass (std::string);

		private:
			Graph3* _graph;
			Optimizer3* _opt;
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
            std::map<int, int> _frameToGraphMap;

            struct Frame {
                nav_msgs::Odometry odom;
                sensor_msgs::PointCloud2 cloud;
                std::vector<darknet_ros_msgs::BoundingBox> bb;
            };

            std::queue<Frame> _FrameQueue;
            std::mutex _FrameMutex;

            tf::TransformListener *_listener;
	};
}

#endif
