#include <mutex>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "layout_prediction/system.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/frame.h"

WallDetector::WallDetector (){}

void WallDetector::run ()
{
    while (1)
    {
        if (_framesQueue.empty())
            continue;
        std::unique_lock <std::mutex> lock (_framesQueueMutex);
        detect (_framesQueue.front());
        _framesQueue.pop ();
        lock.unlock ();
    }
}

void WallDetector::attachTo (System* system)
{
    _system = system;
}

void WallDetector::detect (Frame* frame)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = frame->getCloud();
    Pose* pose = frame->getPose();

	pcl::PointCloud<pcl::PointXYZ>::Ptr _laser (new pcl::PointCloud<pcl::PointXYZ>);

	int height = static_cast<int>(cloud->height/4);
	for(int i = 0; i < cloud->width; i++)
		_laser->push_back(cloud->at(i,height));

	_laser->header.frame_id = cloud->header.frame_id;
	_laser->header.seq = cloud->header.seq;

	line_fitting (_laser, _lines);
//
//    if (struktur.empty()) 
//        struktur.insert(struktur.end(), lines.begin(), lines.end());
//    else
//        data_asosiasi (lines);
//
//	visualize_walls(struktur);

//	_pub.publish(*_laser);
};

void
WallDetector::line_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<line>& lines)
{
	std::vector<pcl::PointIndices> cluster_indices;
	tf::TransformListener listener;

	// NaN-based clustering
	std::vector<int> in;
	for(size_t i = 0; i < cloud->width; ++i)
	{
		if(pcl::isFinite(cloud->points[i]))
		{
			in.push_back(i);
			continue;
		}

		if(in.size() > 0)
		{
			pcl::PointIndices r;
			r.indices.resize(in.size());
			for(size_t j = 0; j < in.size(); ++j)
				r.indices[j] = in[j];
			cluster_indices.push_back(r);
		}

		in.clear();
	}

	// Concat clusters 
	if(!cluster_indices.empty())
	{
		for(size_t i = 0; i < cluster_indices.size()-1 ; ++i)
		{
			// calculate distance between consecutive clusters
			if(!cluster_indices[i].indices.empty())
			{
				float dist = pcl::euclideanDistance(
						cloud->points[cluster_indices[i].indices.back()],
						cloud->points[cluster_indices[i+1].indices[0]]);
				if(dist < 10)
				{
					cluster_indices[i].indices.reserve(
							cluster_indices[i].indices.size() +
							cluster_indices[i+1].indices.size()); 
					cluster_indices[i].indices.insert(
							cluster_indices[i].indices.end(),
							cluster_indices[i+1].indices.begin(), 
							cluster_indices[i+1].indices.end());
					cluster_indices.erase(cluster_indices.begin() + i);
				}
			}
		}
	}

	
	// For every cluster j ...
	int j = 0;
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
			it != cluster_indices.end();
			++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::const_iterator pit = it->indices.begin(); 
				pit != it->indices.end();
				++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// ... do sequential multi RANSAC
		while(true)
		{

			pcl::ModelCoefficients coefficients;
			pcl::PointIndices inliers;

			pcl::SACSegmentation<pcl::PointXYZ> seg;
			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_LINE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.1);

			seg.setInputCloud(cloud_cluster);
			seg.segment(inliers, coefficients);

			if(inliers.indices.size() == 0)
			{
				PCL_ERROR("No inliers.");
				break;
			}

			if(inliers.indices.size() > 30)
			{
                pcl::PointXYZ c, Start, End, p1, p2;
                c.x = coefficients.values[0];
                c.y = coefficients.values[1];
                c.z = coefficients.values[2];
                Start = cloud_cluster->points[inliers.indices.front()];
                End = cloud_cluster->points[inliers.indices.back()];

                double center2Start = pcl::euclideanDistance (c, Start);
                double center2End = pcl::euclideanDistance (c, End);

                line l;
				l.p.header.frame_id = cloud->header.frame_id;
				l.p.header.seq = cloud->header.seq;
				l.q.header.frame_id = cloud->header.frame_id;
				l.q.header.seq = cloud->header.seq;

				l.p.point.x = coefficients.values[0] - center2Start * coefficients.values[3];
				l.p.point.y = coefficients.values[1] - center2Start * coefficients.values[4];
				l.p.point.z = coefficients.values[2] - center2Start * coefficients.values[5];
				l.q.point.x = coefficients.values[0] + center2End * coefficients.values[3];
				l.q.point.y = coefficients.values[1] + center2End * coefficients.values[4];
				l.q.point.z = coefficients.values[2] + center2End * coefficients.values[5];

                l.p = transformPoint (boost::ref (listener), l.p);
                l.q = transformPoint (boost::ref (listener), l.q);

                l.p.point.z = 0;
                l.q.point.z = 0;

                l.m = calculate_slope (l); 

                // (y2-y1) = m(x2-x1)
                // y2 = m(x2-x1) + y1
                // y2 = m.x2 - m.x1 + y1
                // y2 = m.x2 + c
                // with c = -m.x1 + y1
                l.c = -l.m * l.p.point.x + l.p.point.y;

                l.theta = atan(-1/l.m) * 180/M_PI;
                l.r = l.c/sqrt (l.m * l.m + 1);
                l.fitness = 0.0;
                lines.push_back(l);

			}

			pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = cloud_cluster->begin();
			cloud_cluster->erase(cloud_iter,cloud_iter + inliers.indices.back());
		}

		j++;
	}
}

geometry_msgs::PointStamped 
WallDetector::transformPoint (const tf::TransformListener& listener,geometry_msgs::PointStamped p)
{
	geometry_msgs::PointStamped q;

	try
    {
		listener.waitForTransform ("/openni_rgb_optical_frame","/odom_combined",p.header.stamp,ros::Duration(2.0));
		listener.transformPoint ("/odom_combined", p, q);
	} catch (tf::TransformException ex) 
    {
		ROS_ERROR ("%s",ex.what());
	}

	return q;
}

//int marker_id = 0;
//void visualize_walls (std::vector<line>& lines)
//{
//	visualization_msgs::Marker marker;
//	marker.header.frame_id = "odom_combined";
//
//	marker.id = 0; //marker_id++;
//	marker.type = visualization_msgs::Marker::LINE_LIST;
//
//	marker.action = visualization_msgs::Marker::ADD;
//	marker.scale.x = 0.1;
//    marker.color.a = 1.0;
//
//    for (int i = 0; i < lines.size(); ++i)
//    {
//        geometry_msgs::Point p;
//        geometry_msgs::Point q;
//
//        p = lines[i].p.point;
//        q = lines[i].q.point;
//
//        marker.color.r = p.x;
//        marker.color.g = p.y;
//        marker.color.b = 1.0;
//
//        marker.points.push_back(p);
//        marker.points.push_back(q);
//    }
//
//    marker.lifetime = ros::Duration();
////    pub_marker.publish(marker);
//}



//void data_asosiasi (std::vector<line>& lines)
//{
//    double m_threshold = 10;
//    double c_threshold = 10;
//    double d_threshold = 50;
//
//    for (int i = 0; i < lines.size(); ++i)
//    {
//        for (int j = 0; j < struktur.size(); ++j)
//        {
//            if (    abs(lines[i].m - struktur[j].m) < m_threshold &&
//                    abs(lines[i].c - struktur[j].c) < c_threshold)
//            {
//                // if distance of lines[i] and struktur[j] < d_threshold
//                // merge (lines[i], struktur[j])
//                // else
//                // struktur.push_back(lines[i]);
//                break;
//            }
//
//            if (j == struktur.size() - 1)
//                struktur.push_back(lines[i]);
//        }
//    }
//}

