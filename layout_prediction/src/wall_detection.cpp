#include <ros/ros.h>

#include "layout_prediction/wall_detector.h"
#include "layout_prediction/system.h"
#include "layout_prediction/settings.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

int main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    return 0;
}
