#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub_filtered_cloud;
ros::Publisher pub_marker;
ros::Publisher pub_line_segment;

pcl::PointCloud<pcl::PointXYZ> 
filter_cloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
	int height = (int) cloud.height/4;
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

	for(int i = 0; i < cloud.width; i++)
		filtered_cloud.push_back(cloud(i,height));

	filtered_cloud.header.frame_id = cloud.header.frame_id;

	return filtered_cloud;
}

void generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ> cloud)
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

		// Do sequential multi RANSAC
		while(true)
		{
			std_msgs::Float64MultiArray line_segment;

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
				geometry_msgs::Point p;
				geometry_msgs::Point q;

				p.x = cloud_cluster->points[inliers.indices.front()].z;
				p.y = - cloud_cluster->points[inliers.indices.front()].x;
				p.z = 0;
				q.x = cloud_cluster->points[inliers.indices.back()].z;
				q.y = - cloud_cluster->points[inliers.indices.back()].x;
				q.z = 0;

				line_segment.data.clear();
				line_segment.data.push_back(p.x);
				line_segment.data.push_back(p.y);
//				line_segment.data.push_back(p.z);
				line_segment.data.push_back(q.x);
				line_segment.data.push_back(q.y);
//				line_segment.data.push_back(q.z);

				marker.points.push_back(p);
				marker.points.push_back(q);
			}

			pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = cloud_cluster->begin();
			cloud_cluster->erase(cloud_iter,cloud_iter + inliers.indices.back());

			marker.lifetime = ros::Duration();
			pub_marker.publish(marker);
			pub_line_segment.publish(line_segment);
		}

		j++;
	}
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_point)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input_point,cloud);

	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	filtered_cloud = filter_cloud(cloud);

	generate_lines_ransac(filtered_cloud);
	pub_filtered_cloud.publish(filtered_cloud);
}


int
main (int argc, char** argv)
{
	ros::init(argc,argv,"cloud_proc");
	ros::NodeHandle nh;

	ros::Subscriber sub_point = nh.subscribe ("pointcloud", 1, cloud_cb);

	pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_pointcloud",1);
	pub_marker = nh.advertise<visualization_msgs::Marker> ("line_strip",1);
	pub_line_segment = nh.advertise<std_msgs::Float64MultiArray> ("line_segment",1);

	ros::spin();

	return 0;
}

