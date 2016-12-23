#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <layout_prediction/helpers.h>

ros::Publisher pub_marker;
ros::Publisher pub_filtered_cloud;

void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr&);
void odometry_cb (const nav_msgs::Odometry::ConstPtr&);
std::vector<geometry_msgs::PointStamped> generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ>);

void visualize_walls (std::vector<geometry_msgs::PointStamped>);

// This is container for line_segments representing walls
std::vector<wall> walls;


int
main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	ros::init(argc,argv,"layout_prediction");
	ros::NodeHandle nh;


	ros::Subscriber sub_cloud = nh.subscribe ("cloud", 1, cloud_cb);
	ros::Subscriber sub_odometry = nh.subscribe ("odometry", 1, odometry_cb);

	pub_marker = nh.advertise<visualization_msgs::Marker> ("line_strip",1);
	pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);

	ros::spin();

	return 0;
}

void
cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
	tf::TransformListener listener (ros::Duration(10));

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input_cloud,cloud);

	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

	int height = (int) cloud.height/4;
	for(int i = 0; i < cloud.width; i++)
		filtered_cloud.push_back(cloud(i,height));

	filtered_cloud.header.frame_id = cloud.header.frame_id;
	filtered_cloud.header.seq = cloud.header.seq;
	filtered_cloud.header.stamp = cloud.header.stamp;
	
	std::vector<geometry_msgs::PointStamped> lines;
	lines = generate_lines_ransac (filtered_cloud);

//	for (size_t i = 0; i < lines.size(); ++i)
//	{
//		geometry_msgs::PointStamped r;
//		listener.transformPoint ("odom", lines[i], r);
//	}

/*	for(size_t i = 0; i < lines.size(); i = i + 2)
	{
		line_segment ls;
		ls.p = lines[i].point.p;
		ls.q = lines[i].point.q;

		line l = points_to_line_eq (ls); 

		wall w;
		w.boundaries = ls;
		w.line_equation = l;

		if(walls.empty())
			walls.push_back (w);
		else
		{
			for (size_t j = 0; j < walls.size(); ++j)
			{
				if (w.line_equation.slope - walls[i].line_equation.slope < 0.5 &&
				    w.line_equation.intercept - walls[i].line_equation.intercept < 0.5)
				{


				}


			}

		}

	}
*/	
	// push_back line to walls
	// check for data association
	// input walls to visualize_walls

	visualize_walls(lines);
	
	pub_filtered_cloud.publish(filtered_cloud);
}

void
odometry_cb (const nav_msgs::Odometry::ConstPtr& odometry)
{
}


void visualize_walls (std::vector<geometry_msgs::PointStamped> lines)
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

	if(lines.size() != 0)
	{
		for(size_t i = 0; i < lines.size(); i = i + 2)
		{
			geometry_msgs::Point p;
			geometry_msgs::Point q;
			
			p = lines[i].point;
			q = lines[i+1].point;

			marker.points.push_back(p);
			marker.points.push_back(q);
		}

		marker.lifetime = ros::Duration();
		pub_marker.publish(marker);
	}
}

std::vector<geometry_msgs::PointStamped>
generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ> cloud)
{

	std::vector<geometry_msgs::PointStamped> lines;
	std::vector<pcl::PointIndices> cluster_indices;

	// NaN-based clustering
	std::vector<int> in;
	for(size_t i = 0; i < cloud.width; ++i)
	{
		if(pcl::isFinite(cloud.points[i]))
		{
			in.push_back(i);
			continue;
		}

		if(in.size() > 40)
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
						cloud.points[cluster_indices[i].indices.back()],
						cloud.points[cluster_indices[i+1].indices[0]]);
				if(dist < 0.5)
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
			cloud_cluster->points.push_back(cloud.points[*pit]);
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
			seg.setDistanceThreshold(0.01);

			seg.setInputCloud(cloud_cluster);
			seg.segment(inliers, coefficients);

			if(inliers.indices.size() == 0)
			{
				PCL_ERROR("No inliers.");
				break;
			}

			if(inliers.indices.size() > 30)
			{
				geometry_msgs::PointStamped p;
				geometry_msgs::PointStamped q;

				p.header.frame_id = cloud.header.frame_id;
				p.header.seq = cloud.header.seq;
				q.header.frame_id = cloud.header.frame_id;
				q.header.seq = cloud.header.seq;

				p.point.x = cloud_cluster->points[inliers.indices.front()].z;
				p.point.y = - cloud_cluster->points[inliers.indices.front()].x;
				p.point.z = 0;
				q.point.x = cloud_cluster->points[inliers.indices.back()].z;
				q.point.y = - cloud_cluster->points[inliers.indices.back()].x;
				q.point.z = 0;

				lines.push_back(p);
				lines.push_back(q);
			}

			pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = cloud_cluster->begin();
			cloud_cluster->erase(cloud_iter,cloud_iter + inliers.indices.back());
		}

		j++;
	}

	return lines;
}
