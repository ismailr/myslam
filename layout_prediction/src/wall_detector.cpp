#include <mutex>
#include <stdlib.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "layout_prediction/system.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/frame.h"
#include "layout_prediction/graph.h"

WallDetector::WallDetector (System& system, Graph& graph)
    :_system (&system), _graph (&graph), _previousFrameProcessed (0) {
    _system->setWallDetector (*this);
}

void WallDetector::run ()
{
    while (1)
    {
        if (_system->_framesQueue.empty())
            continue;

        std::unique_lock <std::mutex> lock_frames_queue (_system->_framesQueueMutex);
        for (int i = 0; i < _system->_framesQueue.size(); ++i)
        {
            Frame *frame = _system->_framesQueue.front();

            if (frame->_id == _previousFrameProcessed) // already process this frame
                continue;

            _previousFrameProcessed = frame->_id;
            int useCount = ++frame->_useCount; // copy usercount and use the frame and increment count

            detect (*frame);
            if (useCount == 2) // Tracker already used it, so pop and delete
            {
                _system->_framesQueue.pop ();
                delete frame; // it's a must, otherwise memory leak!
            }
        }
        lock_frames_queue.unlock ();
    }
}

void WallDetector::detect (Frame& frame)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = frame.getCloud();
    Pose* pose = &frame.getPose();

	pcl::PointCloud<pcl::PointXYZ>::Ptr _laser (new pcl::PointCloud<pcl::PointXYZ>);
	_laser->header.frame_id = cloud->header.frame_id;
	_laser->header.seq = cloud->header.seq;

	for(int i = 0; i < cloud->width; i++)
    {
        for(int j = 0; j < cloud->height/5; j++)
        {
            if (i % 10 == 0 && j % 10 == 0)
                _laser->push_back(cloud->at(i,j));
        }
    }


//	int height = static_cast<int>(cloud->height/4);
//	for(int i = 0; i < cloud->width; i++)
//        _laser->push_back(cloud->at(i,height));
//
//    _system->visualize (_laser);
//    line_fitting (_laser, *pose);

    std::vector<Wall*> walls = plane_fitting (_laser, *pose);
    associate_walls (walls); // data association
};

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

void WallDetector::associate_walls (std::vector<Wall*> walls)
{
    std::vector<Wall*> wall_marks = _graph->getAllVertices ();
    if (wall_marks.empty())
    {
        std::unique_lock <std::mutex> lock (_graph->_graphUpdateMutex);
        for (int i = 0; i < walls.size(); ++i)
            _graph->addVertex (walls[i]);
        lock.unlock();
    }

    double rho_threshold = 1.0;
    double theta_threshold = 1.0;

    /*** BRUTE FORCE !!! ***/
    for (int i = 0; i < walls.size(); i++)
    {
        for (int j = 0; i < wall_marks.size (); j++)
        {
            if (    abs (walls[i]->rho() - wall_marks[j]->rho()) < rho_threshold && 
                    abs (walls[i]->theta() - wall_marks[j]->theta()) < theta_threshold)
            {
                // update graph
                break;
            } 

            if (j == wall_marks.size() - 1)
            {
                std::unique_lock <std::mutex> lock (_graph->_graphUpdateMutex);
                _graph->addVertex (walls[i]);
                lock.unlock();
            }
        }
    }
}


void WallDetector::line_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Pose& pose)
{
	std::vector<pcl::PointIndices> cluster_indices;
	tf::TransformListener listener;

    std::vector<Wall*> walls;

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

                l.c = -l.m * l.p.point.x + l.p.point.y;

                l.theta = atan(-1/l.m) * 180/M_PI;
                l.r = l.c/sqrt (l.m * l.m + 1);
                l.fitness = 0.0;

                Eigen::Vector2d p (l.p.point.x, l.p.point.y);
                Eigen::Vector2d q (l.q.point.x, l.q.point.y);

                Wall *wall = new Wall (l.r, l.theta, p, q); 
                wall->setFitness (l.fitness);
                wall->setObserverPose (pose);

//                wall->write (std::cout);
//                std::cout << std::endl;
//                _system->visualize (*wall);
                walls.push_back (wall);

                // for each wall:
                //    do data_association
                //    add/update vertex
                // endfor

			}

			pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = cloud_cluster->begin();
			cloud_cluster->erase(cloud_iter,cloud_iter + inliers.indices.back());
		}

		j++;
	}

    _system->visualize (walls);
}

std::vector<Wall*> WallDetector::plane_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Pose& pose)
{
    std::vector<Wall*> walls;

	tf::TransformListener listener;
    Eigen::Vector3f axis (0.0f,0.0f,1.0f);

    /* *** EUCLIDEAN CLUSTER EXTRACTION ***/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (1);
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    /* *** END OF EUCLIDEAN CLUSTER EXTRACTION ***/

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
            it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_cluster->header.frame_id = cloud->header.frame_id;
        for (std::vector<int>::const_iterator pit = it->indices.begin();
                pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back (cloud->points [*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.02);
        seg.setInputCloud (cloud_cluster);
        seg.setAxis (axis);
        seg.setEpsAngle (10.0f * (M_PI/180.0f));
        seg.segment (*inliers, *coefficients);

        pcl::PointCloud<pcl::PointXYZ>::Ptr _plane (new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < inliers->indices.size(); ++i)
        {
            _plane->push_back (cloud_cluster->points [inliers->indices[i]]);
        }

        _plane->header.frame_id = cloud_cluster->header.frame_id;
        _plane->header.seq = cloud_cluster->header.seq;

        line l;
        l.p.header.frame_id = cloud_cluster->header.frame_id;
        l.p.header.seq = cloud_cluster->header.seq;
        l.q.header.frame_id = cloud_cluster->header.frame_id;
        l.q.header.seq = cloud_cluster->header.seq;

        l.m = - (coefficients->values[0]/coefficients->values[1]);
        l.c = coefficients->values[4]/coefficients->values[1];

        l.theta = atan(-1/l.m) * 180/M_PI;
        l.r = l.c/sqrt (l.m * l.m + 1);
        l.fitness = 0.0;

        l.p.point.x = cloud_cluster->points [inliers->indices.front()].x;
        l.p.point.y = cloud_cluster->points [inliers->indices.front()].y;
        l.p.point.z = cloud_cluster->points [inliers->indices.front()].z;
        l.q.point.x = cloud_cluster->points [inliers->indices.back()].x;
        l.q.point.y = cloud_cluster->points [inliers->indices.back()].y;
        l.q.point.z = cloud_cluster->points [inliers->indices.back()].z;

        l.p = transformPoint (boost::ref (listener), l.p);
        l.q = transformPoint (boost::ref (listener), l.q);

        l.p.point.z = 0;
        l.q.point.z = 0;

        Eigen::Vector2d p (l.p.point.x, l.p.point.y);
        Eigen::Vector2d q (l.q.point.x, l.q.point.y);

        Wall *wall = new Wall (l.r, l.theta, p, q); 
        wall->setFitness (l.fitness);
        wall->setObserverPose (pose);

        walls.push_back (wall);

        _system->visualize (_plane);
        _system->visualize (walls);

        j++;
    }

    return walls;
}
