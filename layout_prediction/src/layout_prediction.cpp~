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
void generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ>&, std::vector<geometry_msgs::PointStamped>&);
geometry_msgs::PointStamped transformPoint (const tf::TransformListener&, geometry_msgs::PointStamped);

void visualize_walls (std::vector<geometry_msgs::PointStamped>);

// This is container for line_segments representing walls
std::vector<wall> walls;

int
main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	ros::init(argc,argv,"layout_prediction");
	ros::NodeHandle nh;

//	tf::TransformListener listener (ros::Duration(10));
//	ros::Timer timer = nh.createTimer (ros::Duration (1.0), boost::bind (&transformPoint, boost::ref (listener)));

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
//	filtered_cloud.header.stamp = cloud.header.stamp;

	std::vector<geometry_msgs::PointStamped> lines;
	generate_lines_ransac (filtered_cloud,lines);
	visualize_walls(lines);

//	std::cout << std::endl << "Sekuens no. " << cloud.header.seq << std::endl;

	int garis_ke = 1;
	for (size_t i = 0; i < lines.size(); i = i + 2)
	{
		line_segment ls;
		ls.p = lines[i].point;
		ls.q = lines[i+1].point;

		line l;
		l = points_to_line_eq (ls);

//		ROS_INFO ("Garis ke %i, \t grad=%.2f: \t intercept= %.2f", garis_ke++, 
//				l.slope,
//				l.intercept);
		std::cout << l.slope << "\t" << l.intercept << std::endl;
	}
	std::cout << std::endl;

	pub_filtered_cloud.publish(filtered_cloud);
}

void
odometry_cb (const nav_msgs::Odometry::ConstPtr& odometry)
{
}

int marker_id = 0;
void visualize_walls (std::vector<geometry_msgs::PointStamped> lines)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom_combined";
//	marker.header.stamp = ros::Time::now();

	marker.id = marker_id++;
	marker.type = visualization_msgs::Marker::LINE_LIST;

	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.color.a = 1.0;

	if(lines.size() != 0)
	{
		for(size_t i = 0; i < lines.size(); i = i + 2)
		{
			geometry_msgs::Point p;
			geometry_msgs::Point q;
			
			p = lines[i].point; p.z = 0;
			q = lines[i+1].point; q.z = 0;

			marker.color.r = p.x;
			marker.color.g = p.y;
			marker.color.b = 1.0;

			marker.points.push_back(p);
			marker.points.push_back(q);
		}

		marker.lifetime = ros::Duration();
		pub_marker.publish(marker);
	}
}

geometry_msgs::PointStamped
transformPoint (const tf::TransformListener& listener,geometry_msgs::PointStamped p)
{
	geometry_msgs::PointStamped q;

	try{
		listener.waitForTransform ("/openni_rgb_optical_frame","/odom_combined",p.header.stamp,ros::Duration(3.0));
		listener.transformPoint ("/odom_combined", p, q);
//			ROS_INFO ("p: (%.2f,%.2f,%.2f) -----> q: (%.2f,%.2f,%.2f) at time %.2f",
//					lines[i].point.x,lines[i].point.y,lines[i].point.z,
//					q.point.x,q.point.y,q.point.z,
//					q.header.stamp.toSec());
	} catch (tf::TransformException ex) {
		ROS_ERROR ("%s",ex.what());
	}

	return q;
}

void
generate_lines_ransac (pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<geometry_msgs::PointStamped>& lines)
{

	std::vector<pcl::PointIndices> cluster_indices;
	tf::TransformListener listener;

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
//				p.header.stamp = (ros::Time) cloud.header.stamp;
				q.header.frame_id = cloud.header.frame_id;
				q.header.seq = cloud.header.seq;
//				q.header.stamp = (ros::Time) cloud.header.stamp;

				p.point.x = cloud_cluster->points[inliers.indices.front()].x;
				p.point.y = cloud_cluster->points[inliers.indices.front()].y;
				p.point.z = cloud_cluster->points[inliers.indices.front()].z;
				q.point.x = cloud_cluster->points[inliers.indices.back()].x;
				q.point.y = cloud_cluster->points[inliers.indices.back()].y;
				q.point.z = cloud_cluster->points[inliers.indices.back()].z;

				lines.push_back(transformPoint (boost::ref (listener), p));
				lines.push_back(transformPoint (boost::ref (listener), q));

/*				p.point.x = cloud_cluster->points[inliers.indices.front()].z;
				p.point.y = - cloud_cluster->points[inliers.indices.front()].x;
				p.point.z = 0;
				q.point.x = cloud_cluster->points[inliers.indices.back()].z;
				q.point.y = - cloud_cluster->points[inliers.indices.back()].x;
				q.point.z = 0;
				
				lines.push_back(p);
				lines.push_back(q);
*/
			}

			pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = cloud_cluster->begin();
			cloud_cluster->erase(cloud_iter,cloud_iter + inliers.indices.back());
		}

		j++;
	}
}
