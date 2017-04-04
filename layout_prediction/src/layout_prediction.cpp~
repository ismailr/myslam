#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <stdlib.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
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
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <Eigen/Core>

#include "layout_prediction/helpers.h"
#include "layout_prediction/layout_prediction.h"

ros::Publisher pub_marker;
ros::Publisher pub_filtered_cloud;
ros::Publisher pub_image_depth;

void cloud_cb (const sensor_msgs::PointCloud2::Ptr&);
void depth_cb (const sensor_msgs::Image::Ptr&);
void odometry_cb (const nav_msgs::Odometry::Ptr&);
void generate_lines_ransac(pcl::PointCloud<pcl::PointXYZ>&, std::vector<geometry_msgs::PointStamped>&);
geometry_msgs::PointStamped transformPoint (const tf::TransformListener&, geometry_msgs::PointStamped);
int data_associate(std::vector<wall>,line);
void visualize_walls (line_segment_stamped);
void optimalisasi_graf();


// This is container for line_segments representing walls
std::vector<wall> walls;

int
main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	ros::init(argc,argv,"layout_prediction");
	ros::NodeHandle nh;

	ros::Subscriber sub_cloud = nh.subscribe ("cloud", 1, cloud_cb2);
	ros::Subscriber sub_odometry = nh.subscribe ("odometry", 1, odometry_cb);
	ros::Subscriber sub_depth = nh.subscribe ("depth", 1, depth_cb);

	pub_marker = nh.advertise<visualization_msgs::Marker> ("line_strip",1);
	pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);
	pub_image_depth = nh.advertise<sensor_msgs::Image> ("image_depth",1);

	ros::spin();

	return 0;
}

static pcloud::Ptr _cloud; // previous frame
static pcloud::Ptr _cloud_; // current frame
static bool init = true;

struct _w {
    std::vector<line_segment_stamped> _line;
    float slope_average;
    float intercept_average;
};

std::vector<_w> wall_collection;

void cloud_cb2 (const sensor_msgs::PointCloud2::Ptr& input_cloud)
{
    pcloud::Ptr current_pcl (new pcloud);
    pcl::fromROSMsg(*input_cloud,*current_pcl);

    pcloud::Ptr reduced_current_pcl (new pcloud); 
	reduced_current_pcl->header.frame_id = current_pcl->header.frame_id;
	reduced_current_pcl->header.seq = current_pcl->header.seq;

    pcloud::Ptr filtered_current_pcl (new pcloud); 
	filtered_current_pcl->header.frame_id = current_pcl->header.frame_id;
	filtered_current_pcl->header.seq = current_pcl->header.seq;

    reduce_pcl(current_pcl, reduced_current_pcl);
//    filter_pcl(reduced_current_pcl,filtered_current_pcl);
    *filtered_current_pcl = *reduced_current_pcl;

//    _cloud_ = filtered_current_pcl;
//
//    pcloud::Ptr corrected_pcl (new pcloud);
//
//    if (!init)
//    {
//        correct_pcl(_cloud_, _cloud, corrected_pcl);
//    }
//
//    _cloud = _cloud_;
//
//    init = false;

    line_segment_stamped _line;
    line_fitting(filtered_current_pcl,_line);
    line l;
    l = points_to_line_eq (_line);

//    std::ofstream myfile;
//    myfile.open ("/home/ism/lines.txt", std::ios::out | std::ios::app);
//    myfile << l.slope << "\t" << l.intercept << std::endl;
//    myfile.close();

    visualize_walls(_line);

	pub_filtered_cloud.publish(filtered_current_pcl);
}

//void cloud_cb (const sensor_msgs::PointCloud2::Ptr& input_cloud)
//{
//
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	pcl::fromROSMsg(*input_cloud,cloud);
//
//	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
//
//	int height = (int) cloud.height/4;
//	for(int i = 0; i < cloud.width; i++)
//		filtered_cloud.push_back(cloud(i,height));
//
//	filtered_cloud.header.frame_id = cloud.header.frame_id;
//	filtered_cloud.header.seq = cloud.header.seq;
//
//	pcl::PointCloud<pcl::PointXYZ> f;
//
//	/* ****  Filter cloud using statistical outlier removal ****/ 
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//	sor.setInputCloud(filtered_cloud.makeShared());
//	sor.setMeanK(50);
//	sor.setStddevMulThresh(1.0);
//	sor.filter(f); 
	/* ************************ */

	/* **** Voxelized ***** */
//	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_grid;
//	approx_grid.setLeafSize(0.1,0.1,0.1);
//	approx_grid.setInputCloud(filtered_cloud.makeShared());
//	approx_grid.filter(f);
//
//	_cloud_ = filtered_cloud.makeShared();
//	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//	if (!init) 
//	{
//		pcl::NormalDistributionsTransform2D<pcl::PointXYZ,pcl::PointXYZ> ndt;
//		ndt.setTransformationEpsilon(0.01);
//	//	ndt.setStepSize(0.1);
//	//	ndt.setResolution(1.0);
//		ndt.setMaximumIterations(35);
//		ndt.setInputSource(_cloud_);
//		ndt.setInputTarget(_cloud);
//
//		Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ());
//		Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
//		Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
//
//		// ndt.align(*output_cloud,init_guess);
//		ndt.align(*output_cloud);
//
//		Eigen::Matrix4f T = ndt.getFinalTransformation();
////		std::cout << "|" << T(0,0) << "|" << T(0,1) << "|" << T(0,2) << "|" << T(0,3) << std::endl;
////		std::cout << "|" << T(1,0) << "|" << T(1,1) << "|" << T(1,2) << "|" << T(1,3) << std::endl;
////		std::cout << "|" << T(2,0) << "|" << T(2,1) << "|" << T(2,2) << "|" << T(2,3) << std::endl;
////		std::cout << "|" << T(3,0) << "|" << T(3,1) << "|" << T(3,2) << "|" << T(3,3) << std::endl;
////		std::cout << std::endl;
////
//	}
//
//	_cloud = filtered_cloud.makeShared();
//	init = false;
//
//
//	std::vector<geometry_msgs::PointStamped> lines;
//	generate_lines_ransac (filtered_cloud,lines);
////	generate_lines_ransac (f,lines);
////	generate_lines_ransac (*output_cloud,lines);
////	visualize_walls(lines);
//
//	for (size_t i = 0; i < lines.size(); i = i + 2)
//	{
//		line_segment ls;
//		ls.p = lines[i].point;
//		ls.q = lines[i+1].point;
//
//		line l;
//		l = points_to_line_eq (ls);
//
//		wall w;
//		w.boundaries = ls;
//		w.line_equation = l;
//
//		int index = data_associate(walls,l);
//
//		if (index >= 0)
//		{
//
///*			walls[index].line_equation.slope = (walls[index].line_equation.slope 
//								+ w.line_equation.slope)/2;
//			walls[index].line_equation.intercept = (walls[index].line_equation.intercept
//								+ w.line_equation.intercept)/2;
//*/		}
//		else
//			walls.push_back(w); // new wall!
//
//	}
//
//	std::vector<geometry_msgs::PointStamped> lines2;
//
//	if(walls.size() != 0)
//	{
//		for(int i = 0; i < walls.size(); i++)
//		{
//			geometry_msgs::PointStamped _p; 
//			_p.point = walls[i].boundaries.p;
//			lines2.push_back(_p);
//
//			geometry_msgs::PointStamped _q; 
//			_q.point = walls[i].boundaries.q;
//			lines2.push_back(_q);
//		}
//	}
//
//	visualize_walls(lines);
//
//	pub_filtered_cloud.publish(filtered_cloud);
//}

nav_msgs::Odometry::Ptr o (new nav_msgs::Odometry);

void
odometry_cb (const nav_msgs::Odometry::Ptr& odometry)
{
    o = odometry;
}

int marker_id = 0;
void visualize_walls (line_segment_stamped line)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom_combined";
//	marker.header.stamp = ros::Time::now();

	marker.id = marker_id++;
//	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;

	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.color.a = 1.0;

    geometry_msgs::Point p;
    geometry_msgs::Point q;
    
    p = line.p.point; p.z = 0;
    q = line.q.point; q.z = 0;

    marker.color.r = p.x;
    marker.color.g = p.y;
    marker.color.b = 1.0;

    marker.points.push_back(p);
    marker.points.push_back(q);

    marker.lifetime = ros::Duration();
    pub_marker.publish(marker);
}

geometry_msgs::PointStamped
transformPoint (const tf::TransformListener& listener,geometry_msgs::PointStamped p)
{
	geometry_msgs::PointStamped q;

	try{
		listener.waitForTransform ("/openni_rgb_optical_frame","/odom_combined",p.header.stamp,ros::Duration(2.0));
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

geometry_msgs::PoseStamped
transformPose (const tf::TransformListener& listener,geometry_msgs::PoseStamped p)
{
	geometry_msgs::PoseStamped q;

	try{
		listener.waitForTransform ("/base_link","/odom",p.header.stamp,ros::Duration(3.0));
		listener.transformPose ("/base_link", p, q);
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
						cloud.points[cluster_indices[i].indices.back()],
						cloud.points[cluster_indices[i+1].indices[0]]);
				if(dist < 1000000)
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
				geometry_msgs::PointStamped p;
				geometry_msgs::PointStamped q;

				p.header.frame_id = cloud.header.frame_id;
				p.header.seq = cloud.header.seq;
				q.header.frame_id = cloud.header.frame_id;
				q.header.seq = cloud.header.seq;

				// Using point in inliers
/*				p.point.x = cloud_cluster->points[inliers.indices.front()].x;
				p.point.y = cloud_cluster->points[inliers.indices.front()].y;
				p.point.z = cloud_cluster->points[inliers.indices.front()].z;
				q.point.x = cloud_cluster->points[inliers.indices.back()].x;
				q.point.y = cloud_cluster->points[inliers.indices.back()].y;
				q.point.z = cloud_cluster->points[inliers.indices.back()].z;
*/
				float scale = pcl::euclideanDistance(
						cloud_cluster->points[inliers.indices.front()],
						cloud_cluster->points[inliers.indices.back()]);

				// Using coefficients
				p.point.x = coefficients.values[0] - 0.5 * scale * coefficients.values[3];
				p.point.y = coefficients.values[1] - 0.5 * scale * coefficients.values[4];;
				p.point.z = coefficients.values[2] - 0.5 * scale * coefficients.values[5];;
				q.point.x = p.point.x + scale * coefficients.values[3];
				q.point.y = p.point.y + scale * coefficients.values[4];
				q.point.z = p.point.z + scale * coefficients.values[5];

				lines.push_back(transformPoint (boost::ref (listener), p));
				lines.push_back(transformPoint (boost::ref (listener), q));

				// Transformation from openni_rgb_optical_frame to base_frame
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

int data_associate(std::vector<wall> w,line l)
{
	float threshold_slope = 5;
	float threshold_intercept = 5;
	
	float slope_d_min = 1000000000.0;
	float intercept_d_min = 1000000000.0;

	int index = -1;

	for(int i = 0; i < w.size(); i++)
	{
		float slope_d = abs(w[i].line_equation.slope - l.slope);
		float intercept_d = abs(w[i].line_equation.intercept - l.intercept);

		if (slope_d < slope_d_min && intercept_d < intercept_d_min
			&& slope_d < threshold_slope
			&& intercept_d < threshold_intercept)
		{
			slope_d_min = slope_d;
			intercept_d_min = intercept_d;
			index = i;
		}
	}

	return index;
}

int id = 0;
void optimalisasi_graf()
{
    g2o::Isometry3D t;
    t.matrix() << 1,0,0,1,
                  0,1,0,1,
                  0,0,1,1,
                  0,0,0,0;

    g2o::VertexSE3* node = new g2o::VertexSE3;
    node->setId(id++);
    node->setEstimate(t);
    
    node->write(std::cout);

//	SE2 t = SE2(5,5,5);  
//	VertexSE2* node = new VertexSE2;
//	node->setId(id++);
//	node->setEstimate(t);
//
//	SE2 r = SE2(7,7,7);  
//	VertexSE2* noder = new VertexSE2;
//	noder->setId(id++);
//	noder->setEstimate(r);
//
//	node->write(std::cout);
//	noder->write(std::cout);
//
//	SparseOptimizer optimizer;
//
//	typedef BlockSolver< BlockSolverTraits<-1,-1> > SlamBlockSolver;
//	typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//	
//	SlamLinearSolver* linearSolver = new SlamLinearSolver();
//	linearSolver->setBlockOrdering(false);
//	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
//	OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
//
//	optimizer.setAlgorithm(solver);
//	optimizer.addVertex(node);
}

void reduce_pcl(const pcloud::Ptr& in, const pcloud::Ptr& out)
{
	int height = (int) in->height/4;
	for(int i = 0; i < in->width; i++)
		out->push_back((*in)(i,height));

//    out->resize(in->size());
//
//    pcloud::iterator it = in->begin();
//    for(it; it != in->end(); ++it)
//    {
//        if ((*it).y < 0.2 && (*it).y > -0.2)
//            out->insert(it,*it);
//    }
}

void filter_pcl(const pcloud::Ptr& in, const pcloud::Ptr& out)
{

	/* ****  Statistical outlier removal ****/ 
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//	sor.setInputCloud(in);
//	sor.setMeanK(50);
//	sor.setStddevMulThresh(1.0);
//	sor.filter(*out); 
	/* ************************ */

	/* **** Voxelized ***** */
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_grid;
	approx_grid.setLeafSize(0.2,0.2,0.2);
	approx_grid.setInputCloud(in);
	approx_grid.filter(*out);
	/* ************************ */
}

void line_fitting(const pcloud::Ptr& in, line_segment_stamped& line)
{
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;

	tf::TransformListener listener;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setAxis(Eigen::Vector3f::UnitY());

    seg.setInputCloud(in);
    seg.segment(inliers, coefficients);

    if(inliers.indices.size() == 0)
    {
        PCL_ERROR("No inliers.");
    }

    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped q;

    p.header.frame_id = in->header.frame_id;
    p.header.seq = in->header.seq;
    q.header.frame_id = in->header.frame_id;
    q.header.seq = in->header.seq;

//    float scale = 10.0;
    float x1 = in->points[inliers.indices.front()].x;
    float y1 = in->points[inliers.indices.front()].y;
    float z1 = in->points[inliers.indices.front()].z;
    float x2 = in->points[inliers.indices.back()].x;
    float y2 = in->points[inliers.indices.back()].y;
    float z2 = in->points[inliers.indices.back()].z;
    
    float scale = sqrt ( (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

    p.point.x = coefficients.values[0] - 0.5 * scale * coefficients.values[3];
    p.point.y = coefficients.values[1] - 0.5 * scale * coefficients.values[4];;
    p.point.z = coefficients.values[2] - 0.5 * scale * coefficients.values[5];;
    q.point.x = p.point.x + scale * coefficients.values[3];
    q.point.y = p.point.y + scale * coefficients.values[4];
    q.point.z = p.point.z + scale * coefficients.values[5];
//
//    p.point.x = in->points[inliers.indices.front()].x;
//    p.point.y = in->points[inliers.indices.front()].y;
//    p.point.z = in->points[inliers.indices.front()].z;
//    q.point.x = in->points[inliers.indices.back()].x;
//    q.point.y = in->points[inliers.indices.back()].y;
//    q.point.z = in->points[inliers.indices.back()].z;

//    lines.push_back(transformPoint (boost::ref (listener), p));
//    lines.push_back(transformPoint (boost::ref (listener), q));

    line.p = transformPoint (boost::ref (listener), p);
    line.q = transformPoint (boost::ref (listener), q);
}

void correct_pcl(const pcloud::Ptr& source, const pcloud::Ptr& target, const pcloud::Ptr& out)
{

	tf::TransformListener listener;

    /* ***** NDT ***** */ 
    pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.1);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
    ndt.setMaximumIterations(50);
    ndt.setInputSource(source);
    ndt.setInputTarget(target); 
    /* ***** END OF NDT ***** */

    /* ***** ICP ***** */
//    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
//    icp.setInputCloud(source);
//    icp.setInputTarget(target);
    /* ***** END OF ICP ***** */

    tf::Pose pose;
    tf::poseMsgToTF(o->pose.pose, pose);

    double yaw_angle = tf::getYaw (pose.getRotation());

    Eigen::AngleAxisf init_rotation (yaw_angle, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation (
                            o->pose.pose.position.x, 
                            o->pose.pose.position.y,
                            o->pose.pose.position.z);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    pcloud::Ptr result (new pcloud);
//    ndt.align(*result,init_guess);
    ndt.align(*result);
//    icp.align(*result);
//
    Eigen::Matrix4f T = ndt.getFinalTransformation();
    Eigen::Affine3f Tr;
    Tr = T;
    pcl::transformPointCloud (*target,*out,Tr.inverse());

//    std::cout << "Odometri: " << *o << std::endl;
    std::cout << "Odo: " << yaw_angle << " | " 
              << "NDT: " << acos(T(0,0)) << " | "
              << ndt.hasConverged() << std::endl;

//    std::cout << "Converged? " << ndt.hasConverged() << std::endl;
//    std::cout << "Score: " << ndt.getFitnessScore() << std::endl;
}

void get_planes (const pcloud::Ptr& source, std::vector<pcl::ModelCoefficients> coeffs, std::vector<pcl::PointIndices> inliers)
{
    pnormal::Ptr normals (new pnormal); 
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ,pcl::Normal,pcl::Label> seg;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne_normals;
    std::vector<pcl::PlanarRegion<pcl::PointXYZ>,Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;

    ne_normals.setNormalEstimationMethod (ne_normals.COVARIANCE_MATRIX);
//    ne_normals.setMaxDepthChangeFactor (.1f);
//    ne_normals.setNormalSmoothingSize (20.0f);
    ne_normals.setInputCloud (source);
    ne_normals.compute (*normals);

//    seg.setMinInliers (5);
//    seg.setAngularThreshold (.12);
//    seg.setDistanceThreshold (.05);
    seg.setInputCloud(source);
    seg.setInputNormals(normals);
    seg.segment (coeffs, inliers);
//    seg.segment (regions);

    std::cout << coeffs.size() << std::endl;

//    pnormal::Ptr normals (new pnormal);
//    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//    ne.setMaxDepthChangeFactor (0.02f);
//    ne.setNormalSmoothingSize (10.0f);
//    ne.setInputCloud (source);
//    ne.compute (*normals);
//
//    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ> omps;
//    omps.PointCloudNConstPtr n = normals;
//    omps.setInputNormals(const_normals);
//    omps.setInputCloud(source);
//    omps.segment(coeffs, inliers);
//    std::cout << coeffs.size() << std::endl;
}

void depth_cb (const sensor_msgs::Image::Ptr& image)
{
//    Eigen::Vector3d v1, v2;
//    Eigen::Vector3d x(1,0,0);
//    Eigen::Vector3d y(0,1,0);
//
//    for (int i = 1; i < image->height - 1; i++)
//    {
//        for (int j = 1; j < image->width - 1 ; j++)
//        {
//            double depth_xprev = image->data[i * image->width + j - 1];
//            double depth_xnext = image->data[i * image->width + j + 1];
//            double depth_yprev = image->data[(i - 1) * image->width + j];
//            double depth_ynext = image->data[(i + 1) * image->width + j];
//
//            float dzdx = (depth_xnext - depth_xprev)/2.0f;
//            float dzdy = (depth_ynext - depth_yprev)/2.0f;
//
//            float magnitude = sqrt(dzdx*dzdx + dzdy*dzdy + 1.0f);
//            Eigen::Vector3d normal(-dzdx/magnitude,-dzdy/magnitude,1.0f/magnitude);
//        }
//    }
//
	pub_image_depth.publish(image);
}

