#include "layout_prediction/visualizer.h"

namespace MYSLAM {
    Visualizer::Visualizer (ros::NodeHandle nh, System& system, Graph& graph)
        : _system (&system), _graph (&graph), _rosnodehandle (nh)
    {
        _pub_marker = _rosnodehandle.advertise<visualization_msgs::Marker> ("line_strip",1);
        _pub_cloud = _rosnodehandle.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);
    }

    void Visualizer::visualizeWall ()
    {
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom_combined";

        marker.id = 0; //marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;

        std::map<int, Wall::Ptr>::iterator it;
        for (it = wallMap.begin(); it != wallMap.end(); it++)
        {
            geometry_msgs::Point p;
            geometry_msgs::Point q;

            Eigen::Vector2d& wp = it->second->_line.p;
            Eigen::Vector2d& wq = it->second->_line.q;

            p.x = wp.x();
            p.y = wp.y();
            q.x = wq.x();
            q.y = wq.y();

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.points.push_back(p);
            marker.points.push_back(q);
        }

        marker.lifetime = ros::Duration();
        _pub_marker.publish(marker);
    }

    void Visualizer::visualizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        _pub_cloud.publish (*cloud);
    }

}
