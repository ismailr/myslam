#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub_marker;

struct line_segment {
	geometry_msgs::Point p;
	geometry_msgs::Point q;
}; 

struct line {
	float slope;
	float intercept;
}; 

struct wall {
	line_segment boundaries;
	line line_equation;
}; 

inline float calculate_slope (line_segment);
inline line points_to_line_eq (line_segment);
void visualize_walls (const std_msgs::Float64MultiArray::ConstPtr&);

// This is container for line_segments representing walls
std::vector<wall> walls;

void
line_segment_cb (const std_msgs::Float64MultiArray::ConstPtr& points_msg)
{
	if(points_msg->data.size() != 0)
	{
		for(size_t i = 0; i < points_msg->data.size(); i = i + 4)
		{
			geometry_msgs::Point p;
			geometry_msgs::Point q;

			p.x = points_msg->data[i];
			p.y = points_msg->data[i + 1];
			p.z = 0;
			q.x = points_msg->data[i + 2];
			q.y = points_msg->data[i + 3];
			q.z = 0;

			line_segment ls;
			ls.p = p;
			ls.q = q;

			wall w;
			w.boundaries = ls;
			w.line_equation = points_to_line_eq (ls);

			// compare w here to entry in walls
			// to detect similar walls.
			// Must do visual odometry first

			walls.push_back(w);
		}

		visualize_walls(points_msg);
	}
}


int
main (int argc, char** argv)
{
	ros::init(argc,argv,"layout_prediction");
	ros::NodeHandle nh;

	ros::Subscriber sub_point = nh.subscribe ("line_segment", 1, line_segment_cb);

	pub_marker = nh.advertise<visualization_msgs::Marker> ("line_strip",1);

	ros::spin();

	return 0;
}

inline float calculate_slope (line_segment ls)
{
	if(ls.p.x == ls.q.x)
		return 0.0;

	// slope = (y2 - y1)/(x2 - x1)
	float dividend = (ls.q.y - ls.p.y);
	float divisor = (ls.q.x - ls.p.x);

	return dividend/divisor;
}

inline line points_to_line_eq (line_segment ls)
{
	float slope = calculate_slope (ls);

	// Line eq. from two points
	// (y2-y1) = m(x2-x1)
	// y2 = m(x2-x1) + y1
	// y2 = m.x2 - m.x1 + y1
	// y2 = m.x2 + c
	// with c = m.x1 + y1
	
	float intercept = slope * ls.p.x + ls.p.y;

	line calculated_line;
        calculated_line.slope = slope;
	calculated_line.intercept = intercept;
	       
	return calculated_line;
}

void visualize_walls (const std_msgs::Float64MultiArray::ConstPtr& points_msg)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();

	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;

	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	for(size_t i = 0; i < points_msg->data.size(); i = i + 4)
	{
		geometry_msgs::Point p;
		geometry_msgs::Point q;

		p.x = points_msg->data[i];
		p.y = points_msg->data[i + 1];
		p.z = 0;
		q.x = points_msg->data[i + 2];
		q.y = points_msg->data[i + 3];
		q.z = 0;

		marker.points.push_back(p);
		marker.points.push_back(q);
	}

	marker.lifetime = ros::Duration();
	pub_marker.publish(marker);

}
