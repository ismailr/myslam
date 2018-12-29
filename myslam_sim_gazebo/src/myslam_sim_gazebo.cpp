#include <fstream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "myslam_sim_gazebo/simulation.h"
#include "myslam_sim_gazebo/settings.h"

using namespace MYSLAM;

int main (int argc, char** argv)
{
	std::remove ("/home/ism/code/rosws/result/finalpose.dat");
	std::remove ("/home/ism/code/rosws/result/odom.dat");
	std::remove ("/home/ism/code/rosws/result/groundtruth.dat");
	std::remove ("/home/ism/code/rosws/result/objects.dat");
	std::remove ("/home/ism/code/rosws/result/objectsgt.dat");
	std::remove ("/home/ism/code/rosws/result/data.g2o");
	std::remove ("/home/ism/code/rosws/result/da.log");
//	std::remove ("/home/ism/code/rosws/result/threshold.log");

	const char *fileconfig = "/home/ism/code/rosws/src/myslam/myslam_sim_gazebo/src/myslam_sim.cfg";
	MYSLAM::loadConfFile (fileconfig);

	ros::init (argc,argv,"myslam_sim_gazebo");
	ros::NodeHandle nh;

	Graph3 g;
	Optimizer3 o (g);

	MYSLAM::Simulation3 sim (g, o, nh);

	message_filters::Subscriber<nav_msgs::Odometry> subodom (nh, "odom", 1);
	message_filters::Subscriber<myslam_sim_gazebo::LogicalImage> sublogcam (nh, "logcam", 1);

	typedef message_filters::sync_policies::ApproximateTime 
	<nav_msgs::Odometry, myslam_sim_gazebo::LogicalImage> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (100), 
	subodom, sublogcam);

	sync.registerCallback (boost::bind (&MYSLAM::Simulation3::callback, &sim, _1, _2/*, _3, _4, _5, _6, _7*/));

    std::thread t1 (&MYSLAM::Simulation3::thread1, &sim);
    std::thread t2 (&MYSLAM::Simulation3::thread2, &sim);

	ros::spin();

    sim.writeFinalPose();

    t1.join();
    t2.join();

//	double rmse_t = 0.0;
//	double rmse_r = 0.0;
//	sim.calculateRMSE (rmse_t, rmse_r);
//
//	std::ofstream rmsefile;
//	rmsefile.open ("/home/ism/code/rosws/result/data/rmse.dat", std::ios::out | std::ios::app);
//	rmsefile << rmse_t << " " << rmse_r << std::endl;
//	rmsefile.close();
//
//	double percentage = (double)(Simulation::NUM_TRUE_POS + Simulation::NUM_TRUE_NEG) * 100.0/(double)Simulation::NUM_OBSV;
//
//	std::ofstream dafile;
//	dafile.open ("/home/ism/code/rosws/result/data/akurasi.dat", std::ios::out | std::ios::app);
//	dafile 	<< Simulation::NUM_TRUE_POS << " " 
//		<< Simulation::NUM_TRUE_NEG << " " 
//		<< Simulation::NUM_FALSE_POS << " " 
//		<< Simulation::NUM_FALSE_NEG << std::endl;
//	dafile.close();
//
//	std::ofstream timefile;
//	timefile.open ("/home/ism/code/rosws/result/data/time.dat", std::ios::out | std::ios::app);
//	for (auto it = sim.time_map.begin(); it != sim.time_map.end(); it++) {
//		timefile << it->first << " " << it->second << std::endl;
//	}
//	timefile.close();

	return 0;
}
