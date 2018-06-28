#include <ros/ros.h>

#include "layout_prediction/settings.h"
#include "layout_prediction/simulator.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

int main (int argc, char** argv)
{
    const char *fileconfig = "/home/ism/work/code/ros/src/myslam/layout_prediction/src/myslam.cfg";
    MYSLAM::loadConfFile (fileconfig);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    MYSLAM::Simulator sim;
    sim.run();
//    sim.generateRoom();
//    sim.generateLandmark ();

    ros::spin();

    return 0;
}
