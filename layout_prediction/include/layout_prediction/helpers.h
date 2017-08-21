#ifndef __LAYOUT_PREDICTION_HELPERS__
#define __LAYOUT_PREDICTION_HELPERS__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

struct line {
    double m, c, r, theta;
    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped q;
    double fitness;
};

struct line_segment {
	geometry_msgs::Point p;
	geometry_msgs::Point q;
}; 

struct line_segment_stamped {
	geometry_msgs::PointStamped p;
	geometry_msgs::PointStamped q;
};

struct line_eq {
	float slope;
	float intercept;
}; 

double calculate_slope (line);
double calculate_slope (line_segment);
double calculate_slope (line_segment_stamped);
line_eq points_to_line_eq (line_segment);
line_eq points_to_line_eq (line_segment_stamped);

#endif
