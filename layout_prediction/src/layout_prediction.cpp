#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>

void
line_segment_cb (const std_msgs::Float64MultiArray::ConstPtr& line_segment)
{
	if(line_segment->data.size() != 0)
	{
		/*
		 * 1. Cek for duplication
		 * 2. Insert into data structure
		 * 3. Publish marker as line list
		 */
	}
}


int
main (int argc, char** argv)
{
	ros::init(argc,argv,"layout_prediction");
	ros::NodeHandle nh;

	ros::Subscriber sub_point = nh.subscribe ("line_segment", 1, line_segment_cb);

	ros::spin();

	return 0;
}

