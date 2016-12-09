#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

ros::Publisher pub_model_coeff;
ros::Publisher pub_filtered_cloud;
ros::Publisher pub_marker;

void generate_planes_ransac(pcl::PointCloud<pcl::PointXYZ> cloud);
void generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ> cloud);
void generate_object_proposals(pcl::PointCloud<pcl::PointXYZ> cloud);
void visual_odometry(pcl::PointCloud<pcl::PointXYZ> cloud);
pcl::PointCloud<pcl::PointXYZ> filter_cloud(pcl::PointCloud<pcl::PointXYZ> cloud);

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_point)
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input_point,cloud);

	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	filtered_cloud = filter_cloud(cloud);

	generate_lines_ransac(filtered_cloud);
//	generate_planes_ransac(cloud);
//	generate_object_proposals(cloud);
//	visual_odometry(cloud);
//
	pub_filtered_cloud.publish(filtered_cloud);
}

void
scan_cb(const sensor_msgs::LaserScanConstPtr& input_scan)
{
}

int
main (int argc, char** argv)
{
	ros::init(argc,argv,"cloud_proc");
	ros::NodeHandle nh;

	ros::Subscriber sub_point = nh.subscribe ("pointcloud", 1, cloud_cb);

	pub_model_coeff = nh.advertise<pcl_msgs::ModelCoefficients> ("model",1);
	pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_pointcloud",1);
	pub_marker = nh.advertise<visualization_msgs::Marker> ("line_strip",1);

	ros::spin();

	return 0;
}

void generate_planes_ransac(pcl::PointCloud<pcl::PointXYZ> cloud)
{
//	pcl::ModelCoefficients coefficients;
//	pcl::PointIndices inliers;
//
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setDistanceThreshold(0.01);
//
//	seg.setInputCloud(cloud.makeShared());
//	seg.segment(inliers, coefficients);
//
//	pcl_msgs::ModelCoefficients ros_coefficients;
//	pcl_conversions::fromPCL(coefficients, ros_coefficients);
//
//	pub_model_coeff.publish(ros_coefficients);
//
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	ne.setInputCloud(cloud.makeShared());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);


	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
	mps.setMinInliers(100);
	mps.setAngularThreshold(0.017453 * 2.0);
	mps.setDistanceThreshold(0.02);
	mps.setInputNormals(cloud_normals);
	mps.setInputCloud(cloud.makeShared());
	std::vector<pcl::PlanarRegion<pcl::PointXYZ>,Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;
	mps.segmentAndRefine(regions);

//	for(size_t i = 0; i < regions.size(); i++)
//	{
//		Eigen::Vector3f centroid = regions[i].getCentroid();
//		Eigen::Vector4f model = regions[i].getCoefficients();
//		pcl::PointCloud<pcl::PointXYZ> boundary_cloud;
//		boundary_cloud.points = regions[i].getContour();
//		printf("Centroid: (%f, %f, %f)\n Coefficients: (%f, %f, %f, %f)\n Inliers: %d\n",
//				centroid[0], centroid[1], centroid[2],
//				model[0], model[1], model[2], model[3],
//				boundary_cloud.points.size());
//
//	}

			
}

void generate_object_proposals(pcl::PointCloud<pcl::PointXYZ> cloud)
{
}

void visual_odometry(pcl::PointCloud<pcl::PointXYZ> cloud)
{
}

pcl::PointCloud<pcl::PointXYZ> filter_cloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
	int height = (int) cloud.height/2;
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

	for(int i = 0; i < cloud.width; i++)
	{
//		filtered_cloud.push_back(cloud(i,height/2));
		filtered_cloud.push_back(cloud(i,height));
//		filtered_cloud.push_back(cloud(i,(3 * height)/2));
	}

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

	float t = 2.0;

	while(true)
	{

		pcl::ModelCoefficients coefficients;
		pcl::PointIndices inliers;

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.1);

		seg.setInputCloud(cloud.makeShared());
		seg.segment(inliers, coefficients);

		if(inliers.indices.size() == 0)
		{
			PCL_ERROR("No inliers.");
			break;
		}

		if(inliers.indices.size() > 50)
		{
			geometry_msgs::Point p;
			geometry_msgs::Point q;
//			p.x = coefficients.values[2];
//			p.y = - coefficients.values[0];
//			p.z = coefficients.values[1];
//			q.x = p.x + coefficients.values[5]*t;
//			q.y = p.y - coefficients.values[3]*t;
//			q.z = p.z + coefficients.values[4]*t;

			int last = inliers.indices.size() - 1;

			p.x = cloud.points[inliers.indices[0]].z;
			p.y = - cloud.points[inliers.indices[0]].x;
			p.z = cloud.points[inliers.indices[0]].y;
			q.x = cloud.points[inliers.indices[last]].z;
			q.y = - cloud.points[inliers.indices[last]].x;
			q.z = cloud.points[inliers.indices[last]].y;

			marker.points.push_back(p);
			marker.points.push_back(q);
					
		}

		pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = cloud.begin();
			
		for(size_t i = 0; i < inliers.indices.size(); ++i)
		{
		   cloud.erase(cloud_iter + inliers.indices[i]);
		} 

		marker.lifetime = ros::Duration();
		pub_marker.publish(marker);

	}

}
