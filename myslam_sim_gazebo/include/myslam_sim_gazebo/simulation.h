#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <libconfig.h++>

#include "myslam_sim_gazebo/LogicalImage.h"
#include "myslam_sim_gazebo/settings.h"

#include "myslam_system/optimizer.h"

namespace MYSLAM {
	class Simulation {
		public:
			Simulation(Graph&, Optimizer&, ros::NodeHandle&);
			void callback (const nav_msgs::Odometry::ConstPtr& odom,
				const myslam_sim_gazebo::LogicalImage::ConstPtr& logimg);
			void callback2 (const nav_msgs::Odometry::ConstPtr& odom,
				const myslam_sim_gazebo::LogicalImage::ConstPtr& logimg);
			void gzCallback (const gazebo_msgs::ModelStates::ConstPtr& model);
			void addingNoiseToOdom (const SE2&, SE2&);
			void addingNoiseToObject (const myslam_sim_gazebo::LogicalImage::ConstPtr&, myslam_sim_gazebo::LogicalImage::Ptr&);
			void addNodeToMap (Pose::Ptr&, const myslam_sim_gazebo::LogicalImage::ConstPtr&, double);
			void publishTransform (SE2& /* odom */, int /* robot's pose id */, double /* timestamp */);
			void saveData (std::string, SE2&);
			void saveData (std::string, std::string);
			void saveData (std::string, const gazebo_msgs::ModelStates::ConstPtr& models);

			void calculateRMSE(double&, double&);

			static int FRAMECOUNTER; 
			static int KEYFRAMECOUNTER; 
			static int CLASSIDCOUNTER; 
			static int NUM_OBSV;
			static int NUM_TRUE_POS;
			static int NUM_TRUE_NEG;
			static int NUM_FALSE_POS;
			static int NUM_FALSE_NEG;
			static int NUM_OPT;
			static double TIME_OPT;

			std::map<int,double> time_map;

		private:
			Graph* _graph;
			Optimizer* _opt;
			Pose::Ptr _lastPose;
			SE2* _lastOdom;
			SE2* _lastNoisyOdom;
			ros::NodeHandle* _nh;

			std::map<std::string,int> oid; // object_type --> id
			std::map<std::string, int> omap; // object_name --> id

			std::map<int,Eigen::Vector3d> gtpath;

	};
}

#endif
