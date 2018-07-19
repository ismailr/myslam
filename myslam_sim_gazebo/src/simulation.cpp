#include <string>
#include <fstream>
#include <cstdio>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

#include "myslam_sim_gazebo/Model.h"
#include "myslam_sim_gazebo/simulation.h"

#include "myslam_system/data_association.h"
#include "myslam_system/helpers.h"
#include "myslam_visualizer/visualizer.h"

namespace MYSLAM {

	int Simulation::FRAMECOUNTER = 0;
	int Simulation::KEYFRAMECOUNTER = 0;
	int Simulation::CLASSIDCOUNTER = 0;

	Simulation::Simulation(Graph& g, Optimizer& o, ros::NodeHandle& nh) 
		: _graph (&g), _opt (&o), _nh (&nh) {

			Pose::Ptr p (new Pose);
			_lastPose = p;
			_lastOdom = new SE2();
		
		}

	void Simulation::callback (const nav_msgs::Odometry::ConstPtr& odom,
		const myslam_sim_gazebo::LogicalImage::ConstPtr& logimg)
	{
		static tf::TransformBroadcaster br;

		// odom --> map
		tf::Transform trBaselinkToOdom;
		trBaselinkToOdom.setOrigin (tf::Vector3 (odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
		tf::Quaternion qOdom ( odom->pose.pose.orientation.x,
				odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.w);
		trBaselinkToOdom.setRotation (qOdom);
		br.sendTransform (tf::StampedTransform (trBaselinkToOdom, odom->header.stamp, "odom", "base_link"));

		if (FRAMECOUNTER % 15 != 0) {
			FRAMECOUNTER++;
			return;
		}	

		// current odom and pose
		SE2* currentOdom (new SE2);
		Pose::Ptr currentPose (new Pose);

		// Convert odom to SE2 to Pose::Ptr
		Converter c;
		c.odomToSE2 (odom, *currentOdom);

		if (FRAMECOUNTER == 0)
		{
			_lastOdom = currentOdom;
			_lastPose->_pose = currentOdom->toVector();
			_lastPose->_timestamp = odom->header.stamp.toSec();
			_graph->insertNode (_lastPose);
			FRAMECOUNTER++;
			return;
		}

		SE2 delta = _lastOdom->inverse() * *currentOdom;
		SE2 lastPose; lastPose.fromVector (_lastPose->_pose);  
		currentPose->_pose = (lastPose * delta).toVector();
		currentPose->_timestamp = odom->header.stamp.toSec();
		_graph->insertNode (currentPose);

		std::vector<std::tuple<int, Eigen::Vector3d> > data;

		std::ofstream dafile;
		dafile.open ("/home/ism/data/code/rosws/result/da.log", std::ios::out | std::ios::app);
		dafile << "DETECTED OBJ: ";
		for (int i = 0; i < logimg->models.size(); i++) {

			if (oid.empty() || oid.count (logimg->models[i].type) == 0)
				oid [logimg->models[i].type] = CLASSIDCOUNTER++;

			int classid = oid [logimg->models[i].type];
			Eigen::Vector3d measurement;
			measurement.x() = logimg->models[i].pose.position.x;
			measurement.y() = logimg->models[i].pose.position.y;
			measurement.z() = logimg->models[i].pose.position.z;

			data.push_back (std::make_tuple (classid, measurement));

			if (omap.count (logimg->models[i].name) != 0)
				dafile << omap[logimg->models[i].name] << " ";
			else
				dafile << "-1 ";
		}
		dafile << std::endl;
		DataAssociation da (*_graph);
		std::vector<int> result = da.associate (currentPose, data);
		std::vector<int> obj2show;

		dafile << "DASSOCIATION: ";
		for (int i = 0; i < result.size(); i++) {
			Object::Ptr o;
			if (result[i] == -1) {
				Object::Ptr ob (new Object);
				o = ob;
				o->_classid = std::get<0>(data[i]);
				o->_type = logimg->models[i].type;

				SE2 p; p.fromVector (currentPose->_pose);
				SE2 m; m.fromVector (std::get<1>(data[i]));
				o->_pose = (p*m).toVector();
				o->_seenBy.insert (currentPose->_id);
				currentPose->_detectedObjects.insert (o->_id);
				_graph->insertNode (o);
				omap [logimg->models[i].name] = o->_id;
				dafile << "-1 ";
				obj2show.push_back (o->_id);
			    } else {
				o = _graph->_objectMap[result[i]];
				o->_active = true;
				o->_seenBy.insert (currentPose->_id);
				currentPose->_detectedObjects.insert (o->_id);
				_graph->_activeObjects.insert (o->_id);
				dafile << result[i] << " ";
				obj2show.push_back (o->_id);
			    }
				
			    std::tuple<int, int> e (currentPose->_id, o->_id);
			    _graph->insertPoseObjectEdge (e, std::get<1>(data[i]));
		}
		dafile << std::endl;
		dafile << "---------------------------------------------" << std::endl;
		dafile.close();

		// odom --> map
		tf::Transform trOdomToMap;
		trOdomToMap.setOrigin (tf::Vector3 (currentPose->_pose.x(), currentPose->_pose.y(), 0.0));
		tf::Quaternion q;
		q.setRPY (0, 0, currentPose->_pose.z());
		trOdomToMap.setRotation (q);
		br.sendTransform (tf::StampedTransform (trOdomToMap, odom->header.stamp, "map", "odom"));

		int N = _graph->_activePoses.size();
	//    	int M = graph._activeWalls.size();
		int O = _graph->_activeObjects.size();
		int P = _graph->_activePoseObjectEdges.size();
		int E = _graph->_activeEdges.size() + N - 1 + P;
	//    	int V = 3*N + 2*M + 3*O;

		Visualizer v (*_nh, *_graph);
		v.visualizeObjects (obj2show);
		if (KEYFRAMECOUNTER % 3 == 0)
		{
			_opt->localOptimize();
			v.visualizeObjects (obj2show);
		}

		_lastOdom = currentOdom;

		if (_graph->_poseMap.count (currentPose->_id) != 0)
			_lastPose = _graph->_poseMap [currentPose->_id];
		else
			_lastPose = currentPose;

		std::ofstream posefile;
		posefile.open ("/home/ism/data/code/rosws/result/odom.dat", std::ios::out | std::ios::app);
		posefile << currentOdom->toVector().transpose() << std::endl;
		posefile.close();

		KEYFRAMECOUNTER++;
	}

	void Simulation::gzCallback (const gazebo_msgs::ModelStates::ConstPtr& model) {
		Visualizer v (*_nh, *_graph);
		v.visualizeObjectsGT (model);
	}
}
