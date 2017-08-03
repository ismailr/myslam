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
#include <pr2_mechanism_controllers/BaseOdometryState.h>

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
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <Eigen/Core>

ros::Publisher pub_marker;
ros::Publisher pub_filtered_cloud;
ros::Publisher pub_image_depth;

struct line
{
    double m, c, r, theta;
    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped q;
    double fitness;
};

nav_msgs::Odometry::Ptr o (new nav_msgs::Odometry);

void cloud_cb (const sensor_msgs::PointCloud2::Ptr&);
void depth_cb (const sensor_msgs::Image::Ptr&);
void odometry_cb (const nav_msgs::Odometry::Ptr&);
void line_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<line>&);
geometry_msgs::PointStamped transformPoint (const tf::TransformListener&, geometry_msgs::PointStamped);
void visualize_walls (std::vector<line>&);
void optimalisasi_graf();
double calculate_slope (line);
void data_asosiasi (std::vector<line>&);

void action_cb (const pr2_mechanism_controllers::BaseOdometryState::Ptr&);

int main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	ros::init(argc,argv,"layout_prediction");
	ros::NodeHandle nh;

	ros::Subscriber sub_cloud = nh.subscribe ("cloud", 1, cloud_cb);
	ros::Subscriber sub_odometry = nh.subscribe ("odometry", 1, odometry_cb);
	ros::Subscriber sub_depth = nh.subscribe ("depth", 1, depth_cb);
	ros::Subscriber sub_odom = nh.subscribe ("action", 1, action_cb);

	pub_marker = nh.advertise<visualization_msgs::Marker> ("line_strip",1);
	pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);
	pub_image_depth = nh.advertise<sensor_msgs::Image> ("image_depth",1);

	ros::spin();

	return 0;
}

std::vector<line> struktur;
void cloud_cb (const sensor_msgs::PointCloud2::Ptr& input_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input_cloud, *current_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr laser (new pcl::PointCloud<pcl::PointXYZ>);

	int height = (int) current_cloud->height/4;
	for(int i = 0; i < current_cloud->width; i++)
		laser->push_back(current_cloud->at(i,height));

	laser->header.frame_id = current_cloud->header.frame_id;
	laser->header.seq = current_cloud->header.seq;

	std::vector<line> lines;
	line_fitting (laser,lines);
//    std::cout << "position: " << o->pose.pose.position << " ";
//    std::cout << "orientation: " << o->pose.pose.orientation << std::endl;
//    for (int i = 0; i < lines.size(); ++i)
//        std::cout << "line: (" << lines[i].r << "," << lines[i].theta << ")" << std::endl; 
//    std::cout << std::endl;

    if (struktur.empty()) 
        struktur.insert(struktur.end(), lines.begin(), lines.end());
    else
        data_asosiasi (lines);

    /* 
     * Node untuk landmark sudah ada, yaitu di std::vector<line> lines;
     * Node untuk pose, diambil dari odometri lalu gunakan model gerak
     * FRAME    |   DATA ODOMETRI       |   POSE
     * 0        |   0, 0, 0, 0          |   x_0 = 0, y_0 = 0, z_0 = 0, t_0 = 0
     * 1        |   dx1, dy1, dz1, dt1  |   x_1 = f(x_0,dx1,dy1,dz1,dt1), ...
     * 2        |   dx2, dy2, dz2, dt2  |   x_2 = f(x_1,dx2,dy2,dz2,dt2), ... 
     *
     * TODO:
     *  * Sediakan container untuk pose
     *  * Sinkronisasi antara landmark dan pose (odometri)
     *  * Setelah itu baru masukkan ke graph-optimizer
     *  * Tambahkan constraint dari Furlan
     * *** */

	visualize_walls(struktur);

	pub_filtered_cloud.publish(*laser);
}

void odometry_cb (const nav_msgs::Odometry::Ptr& odometry)
{
    o = odometry;
}

int marker_id = 0;
void visualize_walls (std::vector<line>& lines)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom_combined";

	marker.id = 0; //marker_id++;
	marker.type = visualization_msgs::Marker::LINE_LIST;

	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
    marker.color.a = 1.0;

    for (int i = 0; i < lines.size(); ++i)
    {
        geometry_msgs::Point p;
        geometry_msgs::Point q;

        p = lines[i].p.point;
        q = lines[i].q.point;

        marker.color.r = p.x;
        marker.color.g = p.y;
        marker.color.b = 1.0;

        marker.points.push_back(p);
        marker.points.push_back(q);
    }

    marker.lifetime = ros::Duration();
    pub_marker.publish(marker);
}

geometry_msgs::PointStamped transformPoint (const tf::TransformListener& listener,geometry_msgs::PointStamped p)
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

void line_fitting (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<line>& lines)
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

double calculate_slope (line l)
{
	if(l.p.point.x == l.q.point.x)
		return 0.0;

	// slope = (y2 - y1)/(x2 - x1)
	double dividend = (l.p.point.y - l.q.point.y);
	double divisor = (l.p.point.x - l.q.point.x);

	return dividend/divisor;
}

void depth_cb (const sensor_msgs::Image::Ptr& depth)
{

}

void data_asosiasi (std::vector<line>& lines)
{
    double m_threshold = 10;
    double c_threshold = 10;
    double d_threshold = 50;

    for (int i = 0; i < lines.size(); ++i)
    {
        for (int j = 0; j < struktur.size(); ++j)
        {
            if (    abs(lines[i].m - struktur[j].m) < m_threshold &&
                    abs(lines[i].c - struktur[j].c) < c_threshold)
            {
                // if distance of lines[i] and struktur[j] < d_threshold
                // merge (lines[i], struktur[j])
                // else
                // struktur.push_back(lines[i]);
                break;
            }

            if (j == struktur.size() - 1)
                struktur.push_back(lines[i]);
        }
    }
}

void action_cb (const pr2_mechanism_controllers::BaseOdometryState::Ptr& u)
{
//    std::cout << "vx = " << u->velocity.linear.x << std::endl;
//    std::cout << "vy = " << u->velocity.linear.y << std::endl;
//    std::cout << "wz = " << u->velocity.angular.z << std::endl;
//    std::cout << "=============================================" << std::endl; 
}
