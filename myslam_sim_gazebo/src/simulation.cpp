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
	int Simulation::NUM_OBSV = 0;
	int Simulation::NUM_TRUE_POS = 0;
	int Simulation::NUM_TRUE_NEG = 0;
	int Simulation::NUM_FALSE_POS = 0;
	int Simulation::NUM_FALSE_NEG = 0;
	int Simulation::NUM_OPT = 0;
	double Simulation::TIME_OPT = 0.0;

	Simulation::Simulation(Graph& g, Optimizer& o, ros::NodeHandle& nh) 
		: _graph (&g), _opt (&o), _nh (&nh) {

			Pose::Ptr p (new Pose);
			_lastPose = p;
			_lastOdom = new SE2();
			_lastNoisyOdom = new SE2();
		
		}

	void Simulation::callback (const nav_msgs::Odometry::ConstPtr& odom,
		const myslam_sim_gazebo::LogicalImage::ConstPtr& logimg)
	{
		// odom --> base_link
		static tf::TransformBroadcaster br;
		tf::Transform trOdomToBaselink;
		trOdomToBaselink.setOrigin (tf::Vector3 (odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
		tf::Quaternion qOdom ( odom->pose.pose.orientation.x,
				odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.w);
		trOdomToBaselink.setRotation (qOdom);
		br.sendTransform (tf::StampedTransform (trOdomToBaselink, odom->header.stamp, "odom", "base_link"));

		// Convert odom to SE2 for easiness when dealing with orientation
		SE2 odomSE2;
		Converter c1;
		c1.odomToSE2 (odom, odomSE2);

		// logging odom (this actually is a ground truth)
		std::ofstream gtfile;
		gtfile.open ("/home/ism/code/rosws/result/groundtruth.dat", std::ios::out | std::ios::app);
		gtfile << odomSE2.toVector().transpose() << std::endl;
		gtfile.close();

		// not enough objects in a frame
		// TODO: handle this, should even handle situation when sensor only detect 1 object
		if (logimg->models.size() < 2) {
//			*_lastOdom = odomSE2; 
		       	return;
		}

		// Keyframe is every 15th frame
		if (FRAMECOUNTER % MYSLAM::KEYFRAME_POINT != 0) {
			FRAMECOUNTER++;
//			*_lastOdom = odomSE2; 
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
			_lastNoisyOdom = _lastOdom;
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
		dafile.open ("/home/ism/code/rosws/result/da.log", std::ios::out | std::ios::app);
		dafile << "DETECTED OBJ: ";
		for (int i = 0; i < logimg->models.size(); i++) {

			if (oid.empty() || oid.count (logimg->models[i].type) == 0)
				oid [logimg->models[i].type] = CLASSIDCOUNTER++;

			tf::Quaternion q ( logimg->models[i].pose.orientation.x,
					logimg->models[i].pose.orientation.y,
					logimg->models[i].pose.orientation.z,
					logimg->models[i].pose.orientation.w);
			double r, p, y;
			tf::Matrix3x3 (q).getRPY (r,p,y);

			int classid = oid [logimg->models[i].type];
			Eigen::Vector3d measurement;
			measurement.x() = logimg->models[i].pose.position.x;
			measurement.y() = logimg->models[i].pose.position.y;
			measurement.z() = y;

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
				// wrong data association !!!
				// for debugging purpose
				if (omap.count (logimg->models[i].name) != 0)
					dafile << std::endl << "ALERT!!! OBJECT ALREADY ON MAP" << std::endl;

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
				// wrong data association !!!
				// for debugging purpose
				if (omap[logimg->models[i].name] != result[i]) {
					if (omap[logimg->models[i].name] == 0)
						dafile << std::endl << "ALERT!!! WRONG DA: " << -1 << " <-- " << result[i] << std::endl;
					else
						dafile << std::endl << "ALERT!!! WRONG DA: " << omap[logimg->models[i].name] << " <-- " << result[i] << std::endl; 
				}

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
		tf::Transform trMapToBaselink;
		trMapToBaselink.setOrigin (tf::Vector3 (currentPose->_pose.x(), currentPose->_pose.y(), 0.0));
		tf::Quaternion q;
		q.setRPY (0, 0, currentPose->_pose.z());
		trMapToBaselink.setRotation (q);

		tf::Transform trMapToOdom;
		trMapToOdom = (trOdomToBaselink * trMapToBaselink.inverse()).inverse();
		br.sendTransform (tf::StampedTransform (trMapToOdom, odom->header.stamp, "map", "odom"));

		int N = _graph->_activePoses.size();
	//    	int M = graph._activeWalls.size();
		int O = _graph->_activeObjects.size();
		int P = _graph->_activePoseObjectEdges.size();
		int E = _graph->_activeEdges.size() + N - 1 + P;
	//    	int V = 3*N + 2*M + 3*O;

		Visualizer v (*_nh, *_graph);
//		v.visualizeObjects (obj2show);
		if (KEYFRAMECOUNTER % MYSLAM::OPTIMIZATION_POINT == 0)
		{
			_opt->localOptimize();
			v.visualizeObjects (obj2show);
		}

		_lastOdom = currentOdom;

		if (_graph->_poseMap.count (currentPose->_id) != 0)
			_lastPose = _graph->_poseMap [currentPose->_id];
		else
			_lastPose = currentPose;

		KEYFRAMECOUNTER++;
	}

	void Simulation::gzCallback (const gazebo_msgs::ModelStates::ConstPtr& model) {
		Visualizer v (*_nh, *_graph);
		v.visualizeObjectsGT (model);
		saveData ("/home/ism/code/rosws/result/objectsgt.dat", model);
	}

	void Simulation::addingNoiseToOdom (const SE2& odom, SE2& noisy_odom) {
		
		// noise parameter
//		double a1 = 0.01;
//		double a2 = 5.0 * M_PI/180.0;
//		double a3 = 0.01;
//		double a4 = 5.0 * M_PI/180.0;

		double a1 = MYSLAM::A1;
		double a2 = MYSLAM::A2;
		double a3 = MYSLAM::A3;
		double a4 = MYSLAM::A4;
		double odom_min_xy = MYSLAM::A5;
		double odom_min_theta = MYSLAM::A6;

//		double x1 = _lastOdom->translation().x();
//		double y1 = _lastOdom->translation().y();
//		double t1 = _lastOdom->rotation().angle();
//		double x2 = odom.translation().x();
//		double y2 = odom.translation().y();
//		double t2 = odom.rotation().angle();

		SE2 delta = _lastOdom->inverse() * odom;

//		double dx = x2 - x1;
//		double dy = y2 - y1;
//		double dtrans = sqrt (dx*dx + dy*dy);
//		double drot = g2o::normalize_theta (t2 - t1);
//		double alpha = atan2 (dy,dx);
		
		double dx = delta.translation().x();
		double dy = delta.translation().y();
		double dtrans = sqrt (dx*dx + dy*dy);
		double drot = g2o::normalize_theta (delta.rotation().angle());

		double sdx = odom_min_xy + a1 * dtrans + a2 * std::abs (drot);
		double sdy = sdx;
		double sdt = odom_min_theta + a3 * dtrans + a4 * std::abs (drot);

		double noisex = gaussian_generator<double>(0.0, sdx);
		double noisey = noisex;
		double noiset = gaussian_generator<double>(0.0, sdt);

//		Eigen::Vector3d noise3 (noisex, noisey, noiset);
//
//		Eigen::Vector3d out (	_lastNoisyOdom->translation().x() + (dtrans + noisex) * cos (alpha),
//			       		_lastNoisyOdom->translation().y() + (dtrans + noisey) * sin (alpha),
//				       	_lastNoisyOdom->rotation().angle() + drot + noiset);


		SE2 noise (noisex, noisey, noiset); 
		SE2 noisyDelta = delta * noise;

//		noisy_odom.fromVector (out);
		noisy_odom = *_lastNoisyOdom * noisyDelta;
	}

	void Simulation::callback2 (const nav_msgs::Odometry::ConstPtr& odom,
		const myslam_sim_gazebo::LogicalImage::ConstPtr& logimg)
	{
		clock_t start = clock();
		if (logimg->models.size() < 2) return;

		bool firstFrame = false;
		bool keyframe = false;
		if (FRAMECOUNTER == 0) firstFrame = true;
		if (FRAMECOUNTER % MYSLAM::KEYFRAME_POINT == 0) keyframe = true;

		// Convert odom to SE2 for easiness when dealing with orientation
		SE2 currentOdom;
		Converter c;
		c.odomToSE2 (odom, currentOdom);

		// Pose
		Pose::Ptr currentPose (new Pose);

		if (firstFrame) {
			*_lastOdom = currentOdom;
			*_lastNoisyOdom = currentOdom;
			FRAMECOUNTER++;

			// Adding first pose to graph
			currentPose->_pose = currentOdom.toVector();
			currentPose->_timestamp = odom->header.stamp.toSec();
			_graph->insertNode (currentPose);
			KEYFRAMECOUNTER++;

			_lastPose = currentPose;

			// RMSE
			gtpath[currentPose->_id] = currentOdom.toVector();

			return;
		}

		// Adding noise to odom
		SE2 currentNoisyOdom; 
		addingNoiseToOdom (currentOdom, currentNoisyOdom);

		SE2 delta = _lastNoisyOdom->inverse() * currentNoisyOdom;
		SE2 lastPose; lastPose.fromVector (_lastPose->_pose);  
		currentPose->_pose = (lastPose * delta).toVector();
		currentPose->_timestamp = odom->header.stamp.toSec();

		Visualizer v (*_nh, *_graph);
		v.visualizeObjects(); // before optimization
		if (keyframe) {
			ofstream g;
			g.open("/home/ism/code/rosws/result/threshold.log", std::ios::out | std::ios::app);
			g << "*****" << FRAMECOUNTER << "*****" << std::endl;
			g.close();

			addNodeToMap (currentPose, logimg, odom->header.stamp.toSec());
			if (KEYFRAMECOUNTER % MYSLAM::OPTIMIZATION_POINT == 0 && _graph->_activeObjects.size() != 0) {
			       _opt->localOptimize3();
			       _opt->localOptimize();
			}
//			v.visualizeObjects(); // after optimization
			KEYFRAMECOUNTER++;

			// RMSE
			gtpath[currentPose->_id] = currentOdom.toVector();
		}

		*_lastOdom = currentOdom;
		*_lastNoisyOdom = currentNoisyOdom;

		if (_graph->_poseMap.count (currentPose->_id) != 0)
			_lastPose = _graph->_poseMap [currentPose->_id];
		else
			_lastPose = currentPose;

		FRAMECOUNTER++;

		publishTransform (currentNoisyOdom, currentPose->_id, odom->header.stamp.toSec());
		saveData ("/home/ism/code/rosws/result/groundtruth.dat", currentOdom);
		saveData ("/home/ism/code/rosws/result/odom.dat", currentNoisyOdom);
		saveData ("/home/ism/code/rosws/result/objects.dat", "object");
//		saveData ("/home/ism/code/rosws/result/finalpose.dat", "pose");

//		Visualizer v (*_nh, *_graph);
//		v.visualizeObjectsmyslam_sim_gazebo::LogicalImage&);
		clock_t end = clock();
		time_map[FRAMECOUNTER] = (double)(end-start)/CLOCKS_PER_SEC;
	}

	void Simulation::addNodeToMap (Pose::Ptr& currentPose, const myslam_sim_gazebo::LogicalImage::ConstPtr& _logimg, double timestamp) {

		_graph->insertNode (currentPose);

		myslam_sim_gazebo::LogicalImage::Ptr logimg (new myslam_sim_gazebo::LogicalImage);
		logimg->header = _logimg->header;
		logimg->pose = _logimg->pose;

		addingNoiseToObject (_logimg, logimg);

		if (logimg->models.size() < 3) return;

		std::vector<std::tuple<int, Eigen::Vector3d> > data;

		std::ofstream dafile;
		dafile.open ("/home/ism/code/rosws/result/da.log", std::ios::out | std::ios::app);
		dafile << "*****" << FRAMECOUNTER << "*****" << std::endl;
		dafile << "DETECTED OBJ: ";
		for (int i = 0; i < logimg->models.size(); i++) {

			if (oid.empty() || oid.count (logimg->models[i].type) < 1)
				oid [logimg->models[i].type] = CLASSIDCOUNTER++;

			tf::Quaternion q ( logimg->models[i].pose.orientation.x,
					logimg->models[i].pose.orientation.y,
					logimg->models[i].pose.orientation.z,
					logimg->models[i].pose.orientation.w);
			double r, p, y;
			tf::Matrix3x3 (q).getRPY (r,p,y);

			int classid = oid [logimg->models[i].type];
			Eigen::Vector3d measurement;
			measurement.x() = logimg->models[i].pose.position.x;
			measurement.y() = logimg->models[i].pose.position.y;
			measurement.z() = y;

			data.push_back (std::make_tuple (classid, measurement));

			if (omap.count (logimg->models[i].name) > 0)
				dafile << omap[logimg->models[i].name] << " ";
			else
				dafile << "-1 ";
		}
		dafile << std::endl;
		DataAssociation da (*_graph);
//		std::vector<int> result = da.associate (currentPose, data);
//		std::ofstream df;
//		df.open ("/home/ism/code/rosws/result/da-details.log", std::ios::out | std::ios::app);
//		df << "===" << FRAMECOUNTER << std::endl;
//		df.close();
		std::vector<int> result = da.associateByPPF (currentPose, data);

		dafile << "DASSOCIATION: ";
		Simulation::NUM_OBSV += result.size();
		for (int i = 0; i < result.size(); i++) {
			Object::Ptr o;
			if (result[i] == -1) {
				// wrong data association !!!
				// for debugging purpose
				if (omap.count (logimg->models[i].name) > 0) {
					dafile << std::endl << "ALERT!!! OBJECT ALREADY ON MAP" << std::endl;
					Simulation::NUM_FALSE_NEG++;
				} else {
					Simulation::NUM_TRUE_NEG++;
				}

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
				if (omap.count (logimg->models[i].name) < 1)
					omap [logimg->models[i].name] = o->_id;
				dafile << "(" << o->_id << ")";


			    } else {
				// wrong data association !!!
				// for debugging purpose
				if (omap[logimg->models[i].name] != result[i]) {
					if (omap[logimg->models[i].name] == 0) {
						dafile << std::endl << "ALERT!!! WRONG DA: " << -1 << " <-- " << result[i] << std::endl;
						Simulation::NUM_FALSE_POS++;
					}
					else {
						dafile << std::endl << "ALERT!!! WRONG DA: " << omap[logimg->models[i].name] << " <-- " << result[i] << std::endl; 
						Simulation::NUM_FALSE_POS++;
					}
				} else {
					Simulation::NUM_TRUE_POS++;
				}


				o = _graph->_objectMap[result[i]];
				o->_active = true;
				o->_seenBy.insert (currentPose->_id);
				currentPose->_detectedObjects.insert (o->_id);
				_graph->_activeObjects.insert (o->_id);
				dafile << result[i] << " ";
			    }
				
			    std::tuple<int, int> e (currentPose->_id, o->_id);
			    _graph->insertPoseObjectEdge (e, std::get<1>(data[i]));
		}
		dafile << std::endl;
		dafile << "---------------------------------------------" << std::endl;
		dafile.close();

//		double percentage = (double)(Simulation::NUM_TRUE_POS + Simulation::NUM_TRUE_NEG) * 100.0/(double)Simulation::NUM_OBSV;
//		std::cout << "TP: " << Simulation::NUM_TRUE_POS << " "
//			  << "TN: " << Simulation::NUM_TRUE_NEG << " " 
//			  << "FP: " << Simulation::NUM_FALSE_POS << " "
//			  << "FN: " << Simulation::NUM_FALSE_NEG << " --> " << percentage << "%" << std::endl;
	}


	void Simulation::publishTransform (SE2& odom, int id, double timestamp) {

		// Broadcast transformations
		static tf::TransformBroadcaster br;

		// odom --> base_link
		tf::Transform trOdomToBaselink;
		trOdomToBaselink.setOrigin (tf::Vector3 (odom.translation().x(), odom.translation().y(), 0.0));
		tf::Quaternion qq;
		qq.setRPY (0, 0, odom.rotation().angle());
		trOdomToBaselink.setRotation (qq);
		br.sendTransform (tf::StampedTransform (trOdomToBaselink, ros::Time (timestamp), "odom", "base_link"));

		// map --> odom
		if (_graph->_poseMap.count (id) > 0) {
			Pose::Ptr pose = _graph->_poseMap [id];

			tf::Transform trMapToBaselink;
			trMapToBaselink.setOrigin (tf::Vector3 (pose->_pose.x(), pose->_pose.y(), 0.0));
			tf::Quaternion q;
			q.setRPY (0, 0, pose->_pose.z());
			trMapToBaselink.setRotation (q);

			tf::Transform trMapToOdom;
			trMapToOdom = (trOdomToBaselink * trMapToBaselink.inverse()).inverse();
			br.sendTransform (tf::StampedTransform (trMapToOdom, ros::Time (timestamp), "map", "odom"));
		}
	}

	void Simulation::saveData (std::string path, SE2& data) {

		std::ofstream f;
		f.open (path, std::ios::out | std::ios::app);
		f << data.toVector().transpose() << std::endl;
		f.close();
	}

	void Simulation::saveData (std::string path, std::string type) {

		std::ofstream f;
		f.open (path, std::ios::out); 

		if (type == "object") {
			for (auto it = _graph->_objectMap.begin(); it != _graph->_objectMap.end(); it++) {
				f << it->first << " " << it->second->_pose.transpose() << std::endl;
			}
		} else if (type == "pose") {
			for (auto it = _graph->_poseMap.begin(); it != _graph->_poseMap.end(); it++) {
				f << it->first << " " << it->second->_pose.transpose() << std::endl;
			}
		}

		f.close();
	}

	void Simulation::saveData (std::string path, const gazebo_msgs::ModelStates::ConstPtr& o) {

		std::ofstream f;
		f.open (path, std::ios::out); 

		for (int i = 0; i < o->name.size(); i++) {

			if (o->name[i] == "ground_plane" || o->name[i] == "robot") continue;

			tf::Quaternion q ( o->pose[i].orientation.x,
					o->pose[i].orientation.y,
					o->pose[i].orientation.z,
					o->pose[i].orientation.w);
			double r, p, y;
			tf::Matrix3x3 (q).getRPY (r,p,y);

			Eigen::Vector3d pose (	o->pose[i].position.x,
						o->pose[i].position.y,
						y);

			f << pose.transpose() << std::endl;
		}

		f.close();
	}

	void Simulation::addingNoiseToObject (const myslam_sim_gazebo::LogicalImage::ConstPtr& _logimg,
			myslam_sim_gazebo::LogicalImage::Ptr& logimg) {
			

		for (auto it = _logimg->models.begin(); it != _logimg->models.end(); it++) {

			double x = it->pose.position.x;
			double y = it->pose.position.y;

			double range = sqrt (x*x + y*y);
			double bearing = atan2 (y,x);

//			double std_r = MYSLAM::B1 * range; 
//			double std_b = MYSLAM::B2 * bearing;
			double std_r = MYSLAM::B1;
			double std_b = MYSLAM::B2;

			double range_noise = gaussian_generator<double>(0.0, std_r);
			double bearing_noise = gaussian_generator<double>(0.0, std_b);

			range += range_noise;
			bearing += bearing_noise;

			range = std::max (0.0, range);
			bearing = g2o::normalize_theta (bearing);

			x = range * cos (bearing);
			y = range * sin (bearing);

			double orientation_noise = gaussian_generator<double>(0.0, MYSLAM::B3);

			tf::Quaternion q (
				it->pose.orientation.x,	
				it->pose.orientation.y,	
				it->pose.orientation.z,	
				it->pose.orientation.w);
			double roll, pitch, yaw;
			tf::Matrix3x3 (q).getRPY (roll,pitch,yaw);

			yaw += orientation_noise;
			yaw = g2o::normalize_theta (yaw);
			q.setRPY (roll,pitch,yaw);

			Eigen::Vector2d p2 (x, y);
			double d = (double) p2.norm();
			double theta = normalize_theta (atan2 (y, x));

			// limit distance and orientation view of logical camera
			if (d < 5.0 && std::fabs(theta) < 1.396) {

			       	logimg->models.push_back(*it);
				logimg->models.back().pose.position.x = x;
				logimg->models.back().pose.position.y = y;
				logimg->models.back().pose.orientation.x = q.x();
				logimg->models.back().pose.orientation.y = q.y();
				logimg->models.back().pose.orientation.z = q.z();
				logimg->models.back().pose.orientation.w = q.w();
			}
		}
	}

	void Simulation::calculateRMSE (double& transl, double& rot) {

		double cum_transl_error_squared = 0.0;
		double cum_rot_error_squared = 0.0;

		for (auto it = gtpath.begin(); it != gtpath.end(); it++) {

			Eigen::Vector2d transl_truth (it->second.x(), it->second.y()); 
			Eigen::Vector2d transl_est (_graph->_poseMap[it->first]->_pose.x(), _graph->_poseMap[it->first]->_pose.y());

			cum_transl_error_squared += (transl_truth - transl_est).squaredNorm();

			double rot_truth = it->second.z();
			double rot_est = _graph->_poseMap[it->first]->_pose.z();
			double rot_error = g2o::normalize_theta (rot_truth - rot_est);

			cum_rot_error_squared += (rot_error*rot_error);
		}

		int N = gtpath.size();
		transl = sqrt((double)(cum_transl_error_squared/N));
		rot = sqrt ((double)(cum_rot_error_squared/N));
	}
}
