#include "myslam_sim_gazebo/simulation.h"

#include "myslam_system/optimizer.h"

using namespace MYSLAM;

int main (int argc, char** argv)
{
	ros::init (argc,argv,"objects_truth");
	ros::NodeHandle nh;

	Graph g;
	Optimizer o (g);

	MYSLAM::Simulation sim (g, o, nh);

	// Subscribe to gazebo/ModelStates
	ros::Subscriber subGazeboModelStates = nh.subscribe ("gzmodelstates", 100, &MYSLAM::Simulation::gzCallback, &sim);

	ros::spin();

	return 0;
}
