#include <mutex>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <limits>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "layout_prediction/system.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/wall_measurement.h"
#include "layout_prediction/settings.h"

WallDetector2::WallDetector2(System2& system, Graph2& graph) 
    :_method(WallDetector2::USE_LINE_FITTING),
    _system (&system),
    _graph (&graph)
{
    _system->set_wall_detector (*this);
}

void WallDetector2::detect(Pose2::Ptr& pose, const PointCloud::Ptr cloud, std::set<int>& walls)
{
    PointCloud _preparedCloud;
    prepare_cloud (_preparedCloud, cloud);

    PointCloudCluster _pointCloudCluster;
    cluster_cloud (_pointCloudCluster, _preparedCloud);

    if (_method == WallDetector2::USE_LINE_FITTING)
    {
        for (PointCloudCluster::iterator it = _pointCloudCluster.begin();
                it != _pointCloudCluster.end(); ++it)
            line_fitting (**it, pose, walls);
    }
    else if (_method == WallDetector2::USE_PLANE_FITTING)
    {

        for (PointCloudCluster::iterator it = _pointCloudCluster.begin();
                it != _pointCloudCluster.end(); ++it)
        {
            Eigen::Vector2d localMeasurement = plane_fitting (**it);
            Eigen::Vector2d globalMeasurement = inverse_measurement (localMeasurement, pose); 
            Wall2::Ptr w = _graph->createWall();
            w->setRho(globalMeasurement[1]);
            w->setTheta(globalMeasurement[0]);
            _system->visualize<Wall2::Ptr> (w);

            w = _graph->data_association(w); 
            pose->insert_detected_wall (w);

            WallMeasurement2::Ptr wm = _graph->createWallMeasurement();
            wm->vertices()[0] = pose.get();
            wm->vertices()[1] = w.get();
            double measurementData[2] = {localMeasurement[0],localMeasurement[1]};
            wm->setMeasurementData (measurementData);
            Eigen::Matrix<double, 2, 2> inf;
            inf.setIdentity();
//            inf << 100.0, 0.0, 0.0, 100.0;
            wm->information () = inf;
        }

    }
}

void WallDetector2::prepare_cloud (PointCloud& _preparedCloud, const PointCloud::Ptr cloud)
{
    const int CLOUD_DIVISOR = 5;
    int width = cloud->width;
    int height = static_cast<int>(cloud->height/CLOUD_DIVISOR);

    if (_method == WallDetector2::USE_LINE_FITTING)
        for(int i = 0; i < width; i++)
            _preparedCloud.push_back(cloud->at(i,height));

    else if (_method == WallDetector2::USE_PLANE_FITTING)
    {
        const int WIDTH_JUMP = 10;
        const int HEIGHT_JUMP = 10;

        for(int i = 0; i < width; i++)
            for(int j = 0; j < height; j++)
                if (i % WIDTH_JUMP == 0 && j % HEIGHT_JUMP == 0)
                    _preparedCloud.push_back(cloud->at(i,j));
    }
}

void WallDetector2::line_fitting (std::vector<Eigen::Vector2d>& lines, PointCloud& _preparedCloud)
{
    while(true)
    {
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);

        seg.setInputCloud(_preparedCloud.makeShared());
        seg.segment(inliers, coefficients);

        if(inliers.indices.size() == 0)
        {
            PCL_ERROR("No inliers.");
            break;
        }

        if(inliers.indices.size() > 50)
        {
            double m;
            coefficients.values[3] == 0 ? m = std::numeric_limits<double>::max() : m = coefficients.values[4]/coefficients.values[3];
            double c = coefficients.values[1]- (m * coefficients.values[0]);

            double x = - (m * c)/(m * m + 1);
            double y = c/(m * m + 1);

            double rho = std::abs (c) /sqrt (m * m + 1);
            double theta = atan2 (y,x); 
            _system->visualize_grad (m, c);
            _system->visualize_rho (rho, theta);

//            Eigen::Vector2d param (theta,rho);
//            lines.push_back (param);

//            double x1 = coefficients.values[0];
//            double y1 = coefficients.values[1];
//            double x2 = coefficients.values[3];
//            double y2 = coefficients.values[4];
//            Eigen::Vector2d p (x1, y1);
//            Eigen::Vector2d q (x2, y2); 
//            q = p + 2.0 * q;
//
//            lines.push_back (p);
//            lines.push_back (q);

            Eigen::Vector2d l (theta, rho);
            lines.push_back (l);
        }

        pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = _preparedCloud.begin();
        _preparedCloud.erase(cloud_iter, cloud_iter + inliers.indices.back());
    }
}

void WallDetector2::line_fitting (PointCloud& _preparedCloud, Pose2::Ptr& pose, std::set<int>& walls)
{
    while(true)
    {
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);

        seg.setInputCloud(_preparedCloud.makeShared());
        seg.segment(inliers, coefficients);


        if(inliers.indices.size() == 0)
        {
            PCL_ERROR("No inliers.");
            break;
        }

        if(inliers.indices.size() > 100)
        {
            double mc_[2];
            double& m_ = mc_[0]; 
            double& c_ = mc_[1]; 
            coefficients.values[3] == 0 ? 
                m_ = std::numeric_limits<double>::max() :
                m_ = coefficients.values[4]/coefficients.values[3];
            c_ = coefficients.values[1]- (m_ * coefficients.values[0]);

            double xx_ = - (m_ * c_)/(m_ * m_ + 1);
            double xy_ = c_/(m_ * m_ + 1);

            double mc[2];
            localToGlobal (mc_, pose, mc);

            double& m = mc[0];
            double& c = mc[1]; 

            double xx = - (m * c)/(m * m + 1);
            double xy = c/(m * m + 1);
            double xxx[2] = {xx, xy}; // intersect point

            Wall3::Ptr w = _graph->createWall3WithId();
            w->setEstimateDataImpl (xxx);

            std::vector<Eigen::Vector3d> points = extract_inliers (inliers, _preparedCloud);
            Eigen::Vector2d p_, q_, p, q; 
            p_(0) = points.front().x();
            p_(1) = points.front().y();
            q_(0) = points.back().x();
            q_(1) = points.back().y();

            double x = pose->estimate().translation().x();
            double y = pose->estimate().translation().y();
            double t = pose->estimate().rotation().angle();
            double cost = cos(t);
            double sint = sin(t);

            p(0) = p_(0)*cost - p_(1)*sint + x;
            p(1) = p_(0)*sint + p_(1)*cost + y;
            q(0) = q_(0)*cost - q_(1)*sint + x;
            q(1) = q_(0)*sint + q_(1)*cost + y;

            w->setpq (p,q);
//            _system->visualize<Wall3::Ptr> (w);
//            _system->visualize_rho (xx, xy);
//
//            double x = pose->estimate().translation().x();
//            double y = pose->estimate().translation().y();
//            double g = pose->estimate().rotation().angle();
//            std::ofstream f;
//            f.open ("/home/ism/tmp/detector1.dat", std::ios::out|std::ios::app);
//            f << "x = " << x << " y = " << y << " p = " << g << std::endl; 
//            f << "m = " << m << " c = " << c << std::endl; 
//            f << "m' = " << m_ << " c' = " << c_ << std::endl; 
//            f << "xx' = " << xx_ << " xy' = " << xy_ << std::endl;
//            f << "xx = " << xx << " xy = " << xy << std::endl;
//            f << std::endl;
//            f.close();

            Wall3::Ptr wfixed = _graph->data_association (w);
            pose->insert_detected_wall3 (wfixed->id());
            walls.insert(wfixed->id());
//            _system->visualize<Wall3::Ptr> (wfixed);

//            WallMeasurement3::Ptr wm = _graph->createWallMeasurement3();
            WallMeasurement3::Ptr wm = _graph->createWallMeasurement3(pose->id(),wfixed->id());
            wm->vertices()[0] = pose.get();
            wm->vertices()[1] = wfixed.get();
            double measurementData[2] = {xx_,xy_};
            wm->setMeasurementData (measurementData);
            Eigen::Matrix<double, 2, 2> inf;
            inf.setIdentity();
            wm->information () = inf;

        }

        pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = _preparedCloud.begin();
        _preparedCloud.erase(cloud_iter, cloud_iter + inliers.indices.back());
    }
}

Eigen::Vector2d WallDetector2::plane_fitting (PointCloud& _preparedCloud)
{
    /* SENSOR FRAME OF REF ****** ROBOT FRAME OF REF 
     *
     *      -y                      z
     *      |                       |
     *      |                       |
     *      |_______ z              |_______ x
     *     /                       /
     *    /                       /
     *   /                       /
     *  x                       -y
     *  
     *  Transformation from SENSOR TO ROBOT FRAME IS
     *  -y --> z
     *   z --> x
     *   x --> -y
     *
     * ************************** */
//    Eigen::Vector3f axis (0.0f,-1.0f,0.0f); // IN SENSOR FRAME
    Eigen::Vector3f axis (0.0f,0.0f,1.0f); // IN ROBOT FRAME

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (_preparedCloud.makeShared());
    seg.setAxis (axis);
    seg.setEpsAngle (10.0f * (M_PI/180.0f));
    seg.segment (*inliers, *coefficients);

    // Gradient and intercept (in robot frame)
    // ax + by + cz + d = 0 
    // for z = 0
    // by = -ax - d
    // y = -(a/b)x - d/b
    // m = -(a/b)
    // c = -(d/b)
    double gradient = - (coefficients->values[0]/coefficients->values[1]);
    double intercept = - (coefficients->values[3]/coefficients->values[1]);
    double rho = std::abs (intercept) /sqrt (pow (gradient, 2) + 1);
    double theta;

    std::ofstream gcfile;
    gcfile.open ("/home/ism/tmp/coeff.dat", std::ios::out | std::ios::app);
    gcfile  << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;
    gcfile.close();

    if (gradient < 0 && intercept < 0)
        theta = atan(-1/gradient) + M_PI;
    else if (gradient > 0 && intercept >= 0)
        theta = atan (-1/gradient) + M_PI;
    else if (gradient == 0 && intercept < 0)
        theta = -M_PI/2;
    else if (gradient == 0 && intercept >= 0)
        theta = M_PI/2;
    else
        theta = atan (-1/gradient);

//    std::ofstream gcfile;
//    gcfile.open ("/home/ism/tmp/gc.dat", std::ios::out | std::ios::app);
//    gcfile << gradient << " " << intercept << std::endl;
//    gcfile.close();

    Eigen::Vector2d wParam (theta, rho);
    return wParam;


//    Wall2::Ptr wall = _graph->createWall();
    // local to global
    // data association
//    wall->setMeasurement (wParam);
//    wall->set_gradient_intercept (gradient, intercept);

    // extract inliers
//    std::vector<Eigen::Vector3d> pointInliers;
//    pointInliers = extract_inliers (inliers, _preparedCloud);
//    wall->set_inliers (pointInliers);

//    walls.push_back (wall);
}

void WallDetector2::cluster_cloud (PointCloudCluster& cluster, PointCloud& _preparedCloud)
{
    typedef std::vector<pcl::PointIndices> PointIndicesCluster;
    PointIndicesCluster _pointIndicesCluster;

    if (_method == WallDetector2::USE_LINE_FITTING)
    {
        // NaN-based clustering
        std::vector<int> in;
        for(size_t i = 0; i < _preparedCloud.width; ++i)
        {
            if(pcl::isFinite(_preparedCloud.points[i]))
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
                _pointIndicesCluster.push_back(r);
            }

            in.clear();
        }

        // Concat clusters 
        if(!_pointIndicesCluster.empty())
        {
            for(size_t i = 0; i < _pointIndicesCluster.size()-1 ; ++i)
            {
                // calculate distance between consecutive clusters
                if(!_pointIndicesCluster[i].indices.empty())
                {
                    float dist = pcl::euclideanDistance(
                            _preparedCloud.points[_pointIndicesCluster[i].indices.back()],
                            _preparedCloud.points[_pointIndicesCluster[i+1].indices[0]]);
                    if(dist < 1.0)
                    {
                        _pointIndicesCluster[i].indices.reserve(
                                _pointIndicesCluster[i].indices.size() +
                                _pointIndicesCluster[i+1].indices.size()); 
                        _pointIndicesCluster[i].indices.insert(
                                _pointIndicesCluster[i].indices.end(),
                                _pointIndicesCluster[i+1].indices.begin(), 
                                _pointIndicesCluster[i+1].indices.end());
                        _pointIndicesCluster.erase(_pointIndicesCluster.begin() + i);
                    }
                }
            }
        }
    }
    else if (_method == WallDetector2::USE_PLANE_FITTING)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (_preparedCloud.makeShared());

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.1);
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (_preparedCloud.makeShared());
        ec.extract (_pointIndicesCluster);
    }

    for (PointIndicesCluster::const_iterator it = _pointIndicesCluster.begin(); 
            it != _pointIndicesCluster.end(); ++it)
    {
        PointCloud::Ptr _cluster (new PointCloud);
        for (std::vector<int>::const_iterator pit = it->indices.begin();
                pit != it->indices.end(); ++pit)
            _cluster->points.push_back (_preparedCloud [*pit]);

        cluster.push_back (_cluster);
    }
}

std::vector<Eigen::Vector3d> WallDetector2::extract_inliers (pcl::PointIndices inliers, PointCloud& _preparedCloud)
{
    std::vector<Eigen::Vector3d> _inliers;
    for (int i = 0; i < inliers.indices.size(); ++i)
    {
        double x = _preparedCloud.points [inliers.indices[i]].x; 
        double y = _preparedCloud.points [inliers.indices[i]].y; 
        double z = _preparedCloud.points [inliers.indices[i]].z;
        Eigen::Vector3d xyz (x, y, z);
        _inliers.push_back (xyz);
    }

    return _inliers;
}

void WallDetector2::localToGlobal (Wall2::Ptr& wall, Pose2::Ptr& pose)
{
    double rho_ = wall->getMeasurement()[1];
    double theta_ = wall->getMeasurement()[0];

    Eigen::Vector3d v = pose->estimate().toVector();
    auto x = v[0];
    auto y = v[1];
    auto alpha = v[2];
//    auto angle = normalize_angle (alpha + theta_);

    double m_, c_;
    theta_ = normalize_angle (theta_);
    theta_ == 0 ? m_ = std::numeric_limits<double>::infinity() : m_ = -1/tan(theta_);
    if (theta_ > 0 && theta_ < M_PI)
        c_ = rho_ * sqrt (m_*m_+1);
    else if (theta_ > M_PI && theta_ < 2 * M_PI)
        c_ = -rho_ * sqrt (m_*m_+1);
    else //(theta_ == 0 || theta_ == M_PI)
        c_ = std::numeric_limits<double>::infinity();

    double m, c;
    m = (sin(alpha) + m_ * cos(alpha))/(cos(alpha) - m_ * sin(alpha));
    c = ((y - m_ * x) * cos(alpha) - (x + m_ * y) * sin(alpha) + c_) / (cos(alpha) - m_ * sin(alpha));

    double xx, yx;
    xx = (-m * c)/(m*m+1);
    yx = c/(m*m+1);

    double rho, theta;
    rho = sqrt (xx*xx + yx*yx);
    theta = atan2(yx,xx);
    if (rho < sqrt(x*x + y*y)) theta = theta + M_PI; 

    // measurement model

    wall->setRho (rho);
    wall->setTheta (theta);
}

Eigen::Vector2d WallDetector2::inverse_measurement (Eigen::Vector2d& localMeasurement, Pose2::Ptr& pose)
{
    double rho_ = localMeasurement[1];
    double theta_ = localMeasurement[0];

    Eigen::Vector3d v = pose->estimate().toVector();
    auto x = v[0];
    auto y = v[1];
    auto alpha = v[2];
//    auto angle = alpha + theta;

    // inverse measurement model
//    rho = std::abs (rho + x * cos (angle) + y * sin (angle));
//    theta = angle;

    double m_, c_;
    theta_ = normalize_angle (theta_);
    theta_ == 0 ? m_ = std::numeric_limits<double>::infinity() : m_ = -1/tan(theta_);
    if (theta_ > 0 && theta_ < M_PI)
        c_ = rho_ * sqrt (m_*m_+1);
    else if (theta_ > M_PI && theta_ < 2 * M_PI)
        c_ = -rho_ * sqrt (m_*m_+1);
    else //(theta_ == 0 || theta_ == M_PI)
        c_ = std::numeric_limits<double>::infinity();

    double m, c;
    m = (sin(alpha) + m_ * cos(alpha))/(cos(alpha) - m_ * sin(alpha));
    c = ((y - m_ * x) * cos(alpha) - (x + m_ * y) * sin(alpha) + c_) / (cos(alpha) - m_ * sin(alpha));

    double xx, yx;
    xx = (-m * c)/(m*m+1);
    yx = c/(m*m+1);

    double rho, theta;
    rho = sqrt (xx*xx + yx*yx);
    theta = atan2(yx,xx);

    Eigen::Vector2d param (theta, rho);
    return param;
}

Eigen::Vector2d WallDetector2::inverse_measurement_from_points (std::vector<Eigen::Vector2d> points, Pose2::Ptr& pose)
{
    Eigen::Vector3d v = pose->estimate().toVector();
    auto x = v[0];
    auto y = v[1];
    auto phi = v[2];

    double _x1 = points[0].x();
    double _y1 = points[0].y();
    double _x2 = points[1].x();
    double _y2 = points[1].y();

    double cosphi = cos(phi);
    double sinphi = sin(phi);

//    double x1 = _x1 * cosphi + _y1 * sinphi - x * cosphi - y * sinphi;
//    double y1 = -_x1 * sinphi + _y1 * cosphi + x * sinphi - y * cosphi;
//    double x2 = _x2 * cosphi + _y2 * sinphi - x * cosphi - y * sinphi;
//    double y2 = -_x2 * sinphi + _y2 * cosphi + x * sinphi - y * cosphi;

    double x1 = _x1 * cosphi - _y1 * sinphi + x; 
    double y1 = _x1 * sinphi + _y1 * cosphi + y;
    double x2 = _x2 * cosphi - _y2 * sinphi + x; 
    double y2 = _x2 * sinphi + _y2 * cosphi + y;

    double m;
    x2 == x1 ? m = std::numeric_limits<double>::max() : m = (y2-y1)/(x2-x1);
    double c = y1 - m * x1;

    double rho = std::abs(c)/sqrt(m*m+1);
    double cx = -(m*c)/(m*m+1);
    double cy = c/(m*m+1);
    double theta = atan2 (cy,cx);


    Eigen::Vector2d param (theta, rho);
    return param;
}

void WallDetector2::localToGlobal (double* mc_, Pose2::Ptr& pose, double* mc)
{
    double x = pose->estimate().translation().x();
    double y = pose->estimate().translation().y();
    double p = pose->estimate().rotation().angle();
    double cosp = cos(p);
    double sinp = sin(p);

    double& m = mc[0];
    double& c = mc[1];
    double& m_ = mc_[0];
    double& c_ = mc_[1];

    m = (sinp +  m_ * cosp)/(cosp - m_ * sinp);
    c = -m * x + y + c_/(cosp - m_ * sinp);
}

namespace MYSLAM {
    WallDetector::WallDetector(System& system) : 
        _system (&system), _method (MYSLAM::WALL_DETECTOR_METHOD)
    {
    }

    void WallDetector::detect(Pose::Ptr& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
            std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >& outWalls)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        prepareCloud (inCloud, cloud);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudset;
        clusterCloud (cloud, cloudset);

        if (_method == LINE_FITTING)
        {
            for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = cloudset.begin();
                    it != cloudset.end(); ++it)
                lineFitting (*it, pose, outWalls);
        }
        else if (_method == PLANE_FITTING)
        {
            for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = cloudset.begin();
                    it != cloudset.end(); ++it)
            {
            }
        }
    }

    void WallDetector::prepareCloud (const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud)
    {
        int width = inCloud->width;
        int height = static_cast<int>(inCloud->height/MYSLAM::WALL_DETECTOR_CLOUD_ROW);

        if (_method == LINE_FITTING)
            for(int i = 0; i < width; i++)
                outCloud->push_back(inCloud->at(i,height));
        else if (_method == PLANE_FITTING) {}
    }

    void WallDetector::clusterCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudset)
    {
        std::vector<pcl::PointIndices> indexset;

        if (_method == LINE_FITTING)
        {
            // NaN-based clustering
            std::vector<int> in;
            for(size_t i = 0; i < inCloud->width; ++i)
            {
                if(pcl::isFinite(inCloud->points[i]))
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
                    indexset.push_back(r);
                }

                in.clear();
            }

            // Concat clusters 
            if(!indexset.empty())
            {
                for(size_t i = 0; i < indexset.size()-1 ; ++i)
                {
                    // calculate distance between consecutive clusters
                    if(!indexset[i].indices.empty())
                    {
                        float dist = pcl::euclideanDistance(
                                inCloud->points[indexset[i].indices.back()],
                                inCloud->points[indexset[i+1].indices.front()]);
                        if(dist < 1.0)
                        {
                            indexset[i].indices.reserve(
                                    indexset[i].indices.size() +
                                    indexset[i+1].indices.size()); 
                            indexset[i].indices.insert(
                                    indexset[i].indices.end(),
                                    indexset[i+1].indices.begin(), 
                                    indexset[i+1].indices.end());
                            indexset.erase(indexset.begin() + i);
                        }
                    }
                }
            }
        }
        else if (_method == PLANE_FITTING)
        {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (inCloud);

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.1);
            ec.setMinClusterSize (100);
            ec.setMaxClusterSize (25000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (inCloud);
            ec.extract (indexset);
        }

        for (std::vector<pcl::PointIndices>::iterator it = indexset.begin(); 
                it != indexset.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::iterator pit = it->indices.begin();
                    pit != it->indices.end(); ++pit)
                cluster->points.push_back ((*inCloud)[*pit]);

            cloudset.push_back (cluster);
        }
    }

    void WallDetector::lineFitting (pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, Pose::Ptr& pose, 
            std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >& outWalls)
    {
        while(true)
        {
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices inliers;

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.1);

            seg.setInputCloud(inCloud);
            seg.segment(inliers, *coefficients);

            if(inliers.indices.size() == 0)
            {
                PCL_ERROR("No inliers.");
                break;
            }

            if(inliers.indices.size() > 100)
            {
                Eigen::Vector2d mc_;
                double& m_ = mc_[0]; 
                double& c_ = mc_[1]; 
                coefficients->values[3] == 0 ? 
                    m_ = std::numeric_limits<double>::infinity() :
                    m_ = coefficients->values[4]/coefficients->values[3];
                c_ = coefficients->values[1]- (m_ * coefficients->values[0]);

                Eigen::Vector2d mc;
                double& m = mc[0];
                double& c = mc[1]; 
                localToGlobal (mc_, pose, mc);

                Eigen::Vector2d xxxy;
                double& xx = xxxy[0];
                double& xy = xxxy[1];
                xx = - (m * c)/(m * m + 1);
                xy = c/(m * m + 1);

                std::vector<Eigen::Vector3d> pts = extractInliers (inliers, inCloud);
                Eigen::Vector2d p_, q_;
                p_.x() = pts.front().x();
                p_.y() = pts.front().y();
                q_.x() = pts.back().x();
                q_.y() = pts.back().y();

                double& x = pose->_pose[0];
                double& y = pose->_pose[1];
                double& t = pose->_pose[2];
                double cost = cos(t);
                double sint = sin(t);

                Eigen::Vector2d p, q;
                p.x() = p_.x()*cost - p_.y()*sint + x;
                p.y() = p_.x()*sint + p_.y()*cost + y;
                q.x() = q_.x()*cost - q_.y()*sint + x;
                q.y() = q_.x()*sint + q_.y()*cost + y;

                // measurement
                Eigen::Vector2d xxxy_;
                double& xx_ = xxxy_[0];
                double& xy_ = xxxy_[1];
//                xx_ = - (m_ * c_)/(m_ * m_ + 1);
//                xy_ = c_/(m_ * m_ + 1);
    
                xx_ = xx*cost + xy*sint - x*cost - y*sint;
                xy_ = -xx*sint + xy*cost + x*sint - y*cost;

                if (m_ != std::numeric_limits<double>::infinity());
                {
                    Wall::Ptr w (new Wall);
                    w->_line.xx = xxxy;
                    w->_line.p = p;
                    w->_line.q = q;
                    w->_cloud = inCloud;
                    w->initParams();

                    std::tuple<Wall::Ptr, Eigen::Vector2d> measurement (w, xxxy_);
                    outWalls.push_back(measurement);

                    _system->getVisualizer()->visualizeWallMeasuredPq(p_,q_, true);
                }
            }

            pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = inCloud->begin();
            inCloud->erase(cloud_iter, cloud_iter + inliers.indices.back());
        }
    }

    void WallDetector::localToGlobal (Eigen::Vector2d& mc_, Pose::Ptr& pose, Eigen::Vector2d& mc)
    {
        double x = pose->_pose[0];
        double y = pose->_pose[1];
        double p = pose->_pose[2];
        double cosp = cos(p);
        double sinp = sin(p);

        double& m = mc[0];
        double& c = mc[1];
        double& m_ = mc_[0];
        double& c_ = mc_[1];

        m = (sinp +  m_ * cosp)/(cosp - m_ * sinp);
        c = -m * x + y + c_/(cosp - m_ * sinp);
    }

    std::vector<Eigen::Vector3d> WallDetector::extractInliers (pcl::PointIndices inliers, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud)
    {
        std::vector<Eigen::Vector3d> pts;
        for (int i = 0; i < inliers.indices.size(); ++i)
        {
            double x = inCloud->points [inliers.indices[i]].x; 
            double y = inCloud->points [inliers.indices[i]].y; 
            double z = inCloud->points [inliers.indices[i]].z;
            Eigen::Vector3d xyz (x, y, z);
            pts.push_back (xyz);
        }

        return pts;
    }

    void WallDetector::detectFromMultiplePoses (
            pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
            std::vector<SE2>& poses,
            std::vector<std::vector<Eigen::Vector2d> >& results) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        prepareCloud (inCloud, cloud);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudset;
        clusterCloud (cloud, cloudset);

        for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = cloudset.begin();
                it != cloudset.end(); ++it)
            lineFittingFromMultiplePoses (*it, poses, results);
    }

    void WallDetector::lineFittingFromMultiplePoses (
            pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
            std::vector<SE2>& poses, 
            std::vector<std::vector<Eigen::Vector2d> >& results) {

        std::vector<pcl::ModelCoefficients::Ptr> models;

        std::cout << "LOOPING" << std::endl;

        while(true) // get lines
        {
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices inliers;

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.1);

            seg.setInputCloud(inCloud);
            seg.segment(inliers, *coefficients);

            if(inliers.indices.size() == 0)
            {
                PCL_ERROR("No inliers.");
                break;
            }

            if(inliers.indices.size() > 100)
                models.push_back (coefficients);

            pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = inCloud->begin();
            inCloud->erase(cloud_iter, cloud_iter + inliers.indices.back());
        }

        // calculate walls w.r.t global
        std::cout << "INVERSE SENSOR MODEL" << std::endl;
        for (int i = 0; i < poses.size(); i++) {

            double x = poses[i].translation().x();
            double y = poses[i].translation().y();
            double t = poses[i].rotation().angle();
            double sint = sin(t);
            double cost = cos(t);

            std::vector<Eigen::Vector2d> res;
            for (int j = 0; j < models.size(); j++) {
                std::cout << "LOOP OVER DETECTED LINES" << std::endl;
                double m_, c_; 
                models[j]->values[3] == 0 ? 
                    m_ = std::numeric_limits<double>::infinity() :
                    m_ = models[j]->values[4]/models[j]->values[3];
                std::cout << "GRAD: " << m_ << std::endl;
                c_ = models[j]->values[1]- (m_ * models[j]->values[0]);

                double m, c;
                m = (sint +  m_ * cost)/(cost - m_ * sint);
                c = -m * x + y + c_/(cost - m_ * sint);

                double u, v; // w.r.t global
                u = - (m * c)/(m * m + 1);
                v = c/(m * m + 1);

                Eigen::Vector2d measurement = poses[i].inverse() * Eigen::Vector2d (u,v);
                std::cout << measurement.transpose() << std::endl;
                res.push_back (measurement);
                std::cout << "DONE" << std::endl;
            }
            results.push_back(res);
            res.clear();
        }
    }
}


