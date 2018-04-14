#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include "layout_prediction/settings.h"
#include "layout_prediction/simulator.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

int main (int argc, char** argv)
{
    const char *fileconfig = "/home/ism/work/code/ros/src/myslam/layout_prediction/src/myslam.cfg";
    MYSLAM::loadConfFile (fileconfig);

	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    MYSLAM::Simulator sim;
    sim.run();
//    sim.generateRoom();
//    sim.generateLandmark ();

    return 0;
}
