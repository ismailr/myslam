
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include "layout_prediction/helpers.h"

long IdGenerator::_id = 0;

double calculate_slope (line_segment ls)
{
	if(ls.p.x == ls.q.x)
		return 0.0;

	// slope = (y2 - y1)/(x2 - x1)
	float dividend = (ls.q.y - ls.p.y);
	float divisor = (ls.q.x - ls.p.x);

	return dividend/divisor;
}

double calculate_slope (line_segment_stamped ls)
{
    line_segment _ls;
    _ls.p = ls.p.point;
    _ls.q = ls.q.point;

	return calculate_slope (_ls);
}

double calculate_slope (line l)
{
    line_segment _ls;
    _ls.p = l.p.point;
    _ls.q = l.q.point;

	return calculate_slope (_ls);
}

line_eq points_to_line_eq (line_segment ls)
{
	float slope = calculate_slope (ls);

	// Line eq. from two points
	// (y2-y1) = m(x2-x1)
	// y2 = m(x2-x1) + y1
	// y2 = m.x2 - m.x1 + y1
	// y2 = m.x2 + c
	// with c = m.x1 + y1
	
	float intercept = slope * ls.p.x + ls.p.y;

	line_eq calculated_line;
    calculated_line.slope = slope;
	calculated_line.intercept = intercept;
	       
	return calculated_line;
}

line_eq points_to_line_eq (line_segment_stamped ls)
{
    line_segment _ls;
    _ls.p = ls.p.point;
    _ls.q = ls.q.point;
	       
	return points_to_line_eq (_ls);
}

void Converter::odomToSE2(const nav_msgs::OdometryConstPtr& odom, SE2& t)
{
    tf::Quaternion q (
                odom->pose.pose.orientation.x,
                odom->pose.pose.orientation.y,
                odom->pose.pose.orientation.z,
                odom->pose.pose.orientation.w
            );
    tf::Matrix3x3 m (q);
    double roll, pitch, yaw;
    m.getRPY (roll, pitch, yaw);

    double odom_x = odom->pose.pose.position.x;
    double odom_y = odom->pose.pose.position.y;
    double odom_theta = yaw;
    Eigen::Vector3d v (odom_x, odom_y, odom_theta);

    t.fromVector (v); 
}

void Converter::odomCombinedToSE2 (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined, SE2& t)
{
    tf::Quaternion q (
                odomcombined->pose.pose.orientation.x,
                odomcombined->pose.pose.orientation.y,
                odomcombined->pose.pose.orientation.z,
                odomcombined->pose.pose.orientation.w
            );
    tf::Matrix3x3 m (q);
    double roll, pitch, yaw;
    m.getRPY (roll, pitch, yaw);

    double odom_x = odomcombined->pose.pose.position.x;
    double odom_y = odomcombined->pose.pose.position.y;
    double odom_theta = yaw;
    Eigen::Vector3d v (odom_x, odom_y, odom_theta);

    t.fromVector (v); 
}

double normalize_angle (double a)
{
    double angle = a;
    if (angle >= 0)
        while (angle > 2 * M_PI)
            angle -= 2 * M_PI;
    else
        while (angle < 0)
            angle += 2 * M_PI;
    return angle;
}

unsigned long int Generator::id = 0;
