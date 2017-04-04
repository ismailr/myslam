#ifndef __LAYOUT_PREDICTION_HELPERS__
#define __LAYOUT_PREDICTION_HELPERS__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

struct line_segment {
	geometry_msgs::Point p;
	geometry_msgs::Point q;
}; 

struct line_segment_stamped {
	geometry_msgs::PointStamped p;
	geometry_msgs::PointStamped q;
};

struct line {
	float slope;
	float intercept;
}; 

struct wall {
	line_segment boundaries;
	line line_equation;
}; 

float calculate_slope (line_segment);
float calculate_slope (line_segment_stamped);
line points_to_line_eq (line_segment);
line points_to_line_eq (line_segment_stamped);

#endif
