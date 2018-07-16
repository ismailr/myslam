#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>

#include "myslam_sim_gazebo/LogicalImage.h"

#include "myslam_system/optimizer.h"

namespace MYSLAM {
	class Simulation {
		public:
			Simulation(Graph&, Optimizer&, ros::NodeHandle&);
			void callback (const nav_msgs::Odometry::ConstPtr& odom,
				const myslam_sim_gazebo::LogicalImage::ConstPtr& logimg);
			void gzCallback (const gazebo_msgs::ModelStates::ConstPtr& model);

			static int FRAMECOUNTER; 
			static int KEYFRAMECOUNTER; 
			static int CLASSIDCOUNTER; 


		private:
			Graph* _graph;
			Optimizer* _opt;
			Pose::Ptr _lastPose;
			SE2* _lastOdom;
			ros::NodeHandle* _nh;

			std::map<std::string,int> oid;
			std::map<std::string, int> omap;
	};
}

#endif
