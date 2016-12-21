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

ros::Publisher pub_marker;
ros::Publisher pub_filtered_cloud;

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

void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr&);
void odometry_cb (const nav_msgs::Odometry::ConstPtr&);
std::vector<geometry_msgs::Point> generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ>);

void visualize_walls (std::vector<geometry_msgs::Point>);

inline float calculate_slope (line_segment);
inline line points_to_line_eq (line_segment);

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
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input_cloud,cloud);

	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

	int height = (int) cloud.height/4;
	for(int i = 0; i < cloud.width; i++)
		filtered_cloud.push_back(cloud(i,height));

	filtered_cloud.header.frame_id = cloud.header.frame_id;
	filtered_cloud.header.seq = cloud.header.seq;
	filtered_cloud.header.stamp = cloud.header.stamp;
	
	std::vector<geometry_msgs::Point> lines;
	lines = generate_lines_ransac (filtered_cloud);
	visualize_walls(lines);
	
	pub_filtered_cloud.publish(filtered_cloud);
}

void
odometry_cb (const nav_msgs::Odometry::ConstPtr& odometry)
{
}

inline float calculate_slope (line_segment ls)
{
	if(ls.p.x == ls.q.x)
		return 0.0;

	// slope = (y2 - y1)/(x2 - x1)
	float dividend = (ls.q.y - ls.p.y);
	float divisor = (ls.q.x - ls.p.x);

	return dividend/divisor;
}

inline line points_to_line_eq (line_segment ls)
{
	float slope = calculate_slope (ls);

	// Line eq. from two points
	// (y2-y1) = m(x2-x1)
	// y2 = m(x2-x1) + y1
	// y2 = m.x2 - m.x1 + y1
	// y2 = m.x2 + c
	// with c = m.x1 + y1
	
	float intercept = slope * ls.p.x + ls.p.y;

	line calculated_line;
        calculated_line.slope = slope;
	calculated_line.intercept = intercept;
	       
	return calculated_line;
}

void visualize_walls (std::vector<geometry_msgs::Point> lines)
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
			
			p = lines[i];
			q = lines[i+1];

			line_segment ls;
			ls.p = p;
			ls.q = q;

			wall w;
			w.boundaries = ls;
			w.line_equation = points_to_line_eq (ls);

			// compare w here to entry in walls
			// to detect similar walls.
			// Must do visual odometry first

			walls.push_back(w);
			marker.points.push_back(p);
			marker.points.push_back(q);
		}

		marker.lifetime = ros::Duration();
		pub_marker.publish(marker);
	}
}

std::vector<geometry_msgs::Point>
generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ> cloud)
{

	std::vector<geometry_msgs::Point> lines;
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
				geometry_msgs::Point p;
				geometry_msgs::Point q;

				p.x = cloud_cluster->points[inliers.indices.front()].z;
				p.y = - cloud_cluster->points[inliers.indices.front()].x;
				p.z = 0;
				q.x = cloud_cluster->points[inliers.indices.back()].z;
				q.y = - cloud_cluster->points[inliers.indices.back()].x;
				q.z = 0;

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
