#ifndef __LAYOUT_PREDICTION_HELPERS__
#define __LAYOUT_PREDICTION_HELPERS__

#include <geometry_msgs/Point.h>

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

float calculate_slope (line_segment);
line points_to_line_eq (line_segment);

#endif
