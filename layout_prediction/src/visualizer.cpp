#include "layout_prediction/visualizer.h"
#include "layout_prediction/settings.h"

namespace MYSLAM {
    int Visualizer::marker_id = 0;

    Visualizer::Visualizer (ros::NodeHandle nh, System& system, Graph& graph)
        : _system (&system), _graph (&graph), _rosnodehandle (nh)
    {
        _pub_wall_measured_pq = _rosnodehandle.advertise<visualization_msgs::Marker> ("wall_measured_pq",1);
        _pub_wall_measured_rt = _rosnodehandle.advertise<visualization_msgs::Marker> ("wall_measured_rt",1);
        _pub_wall_optimized_pq = _rosnodehandle.advertise<visualization_msgs::Marker> ("wall_optimized_pq",1);
        _pub_wall_optimized_rt = _rosnodehandle.advertise<visualization_msgs::Marker> ("wall_optimized_rt",1);
        _pub_cloud = _rosnodehandle.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);
    }

    void Visualizer::visualizeWallOptimizedPq (bool local)
    {
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom_combined";

        marker.ns = "wall";
//        marker.id = 0; //marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;

        std::map<int, Wall::Ptr>::iterator it;
        for (it = wallMap.begin(); it != wallMap.end(); it++)
        {
            marker.id = it->first;
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
        _pub_wall_optimized_pq.publish(marker);
    }

    void Visualizer::visualizeWallOptimizedRt (bool local)
    {
        std::map<int, Wall::Ptr>& wallMap = _graph->_wallMap;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom_combined";

        marker.ns = "wall";
//        marker.id = 0; //marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;

        std::map<int, Wall::Ptr>::iterator it;
        for (it = wallMap.begin(); it != wallMap.end(); it++)
        {
            marker.id = it->first;
            geometry_msgs::Point p;
            geometry_msgs::Point q;

            double& xx = it->second->_line.xx[0];
            double& xy = it->second->_line.xx[1];

            p.x = 0.0;
            p.y = 0.0;
            q.x = xx;
            q.y = xy;

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            marker.points.push_back(p);
            marker.points.push_back(q);
        }

        marker.lifetime = ros::Duration();
        _pub_wall_optimized_rt.publish(marker);
    }

    void Visualizer::visualizeWallMeasuredPq (Eigen::Vector2d& wp, Eigen::Vector2d& wq, bool local)
    {
        visualization_msgs::Marker marker;

        if (local)
            marker.header.frame_id = MYSLAM::PCL_FRAME;
        else
            marker.header.frame_id = "odom_combined";

        marker.id = /* 0;*/ Visualizer::marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;

        geometry_msgs::Point p;
        geometry_msgs::Point q;

        p.x = wp.x();
        p.y = wp.y();
        q.x = wq.x();
        q.y = wq.y();

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.points.push_back(p);
        marker.points.push_back(q);

        marker.lifetime = ros::Duration();
        _pub_wall_measured_pq.publish(marker);
    }

    void Visualizer::visualizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        _pub_cloud.publish (*cloud);
    }

    void Visualizer::visualizeWallMeasuredRt (double xx, double xy, bool local)
    {
        visualization_msgs::Marker marker;
        if (local)
            marker.header.frame_id = MYSLAM::PCL_FRAME;
        else
            marker.header.frame_id = "odom_combined";

        marker.id = 0; //marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;

        geometry_msgs::Point p;
        geometry_msgs::Point q;

        p.x = 0.0; p.y = 0.0;
        q.x = xx;
        q.y = xy;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.points.push_back(p);
        marker.points.push_back(q);

        marker.lifetime = ros::Duration();
        _pub_wall_measured_rt.publish(marker);
    }

}
