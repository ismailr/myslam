#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include "layout_prediction/converter.h"

void Converter::odomToSE2(nav_msgs::OdometryConstPtr& odom, SE2& t)
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


