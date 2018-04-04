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

        cloud->header.frame_id = inCloud->header.frame_id;
        _system->getVisualizer()->visualizeCloud (cloud);
    }

    void WallDetector::detect2(Pose::Ptr& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
            std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> >& outWalls)
    {
        typedef std::tuple<Wall::Ptr, Eigen::Vector2d> w;
        std::vector<std::vector<w> > clusters;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudset;

        const int width = inCloud->width;

        const int vGridStep = 10;
        for (int i = 0; i < vGridStep; i++) {
            int height = static_cast<int>(inCloud->height/(i+1));

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            for(int j = 0; j < width; j++)
                cloud->push_back(inCloud->at(j,height - 1));

            clusterCloud (cloud, cloudset);
            std::vector<w> tmp_ws;
            for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = cloudset.begin();
                    it != cloudset.end(); ++it) {
                lineFitting (*it, pose, tmp_ws);
            }
            if (!tmp_ws.empty())
                clusters.push_back (tmp_ws);

            tmp_ws.clear();
            cloudset.clear();
        }

        std::map<std::tuple<double, double>, std::vector<w> > cluster;
        for (int i = 0; i < clusters.size(); i++) {
            for (int j = 0; j < clusters[i].size(); j++) {
                double _x, _y, x_, y_;
                _x = std::get<0>(clusters[i][j])->_line.xx[0];
                _y = std::get<0>(clusters[i][j])->_line.xx[1];
                
                x_ = round (_x);
                y_ = round (_y);

                std::tuple<double, double> wall = std::make_tuple (x_,y_);
                cluster[wall].push_back (clusters[i][j]);
            }
        }

        std::map<std::tuple<double, double>, std::vector<w> >::iterator cit;
        for (cit = cluster.begin(); cit != cluster.end(); cit++) {
            if (cit->second.size() >= 2) {
                outWalls.push_back (cit->second[1]);
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

    void WallDetector::prepareCloud2 (const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud)
    {
        for (int i = 0; i < inCloud->size(); i++) {
            if (inCloud->points[i].z >= 1.5 && inCloud->points[i].z <= 2.0) {
                pcl::PointXYZ p = inCloud->points[i];
                p.z = 0.0;
                outCloud->push_back (p);
            }
        }
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

                Eigen::Vector2d p, q; // should be shorter --> p = pose * p_;
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
    
                // shorter --> (xx_,xy_) = pose.inverse() * (xx,xy);
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

            if(inliers.indices.size() > 100) {
                models.push_back (coefficients);

                // visualize
                std::vector<Eigen::Vector3d> pts = extractInliers (inliers, inCloud);
                Eigen::Vector2d p_, q_;
                p_.x() = pts.front().x();
                p_.y() = pts.front().y();
                q_.x() = pts.back().x();
                q_.y() = pts.back().y();
                _system->getVisualizer()->visualizeWallMeasuredPq(p_,q_, true);
            }

            pcl::PointCloud<pcl::PointXYZ>::iterator cloud_iter = inCloud->begin();
            inCloud->erase(cloud_iter, cloud_iter + inliers.indices.back());
        }

        // calculate walls w.r.t global
        for (int i = 0; i < poses.size(); i++) {

            double x = poses[i].translation().x();
            double y = poses[i].translation().y();
            double t = poses[i].rotation().angle();
            double sint = sin(t);
            double cost = cos(t);

            std::vector<Eigen::Vector2d> res;
            for (int j = 0; j < models.size(); j++) {
                double m_, c_; 
                models[j]->values[3] == 0 ? 
                    m_ = std::numeric_limits<double>::infinity() :
                    m_ = models[j]->values[4]/models[j]->values[3];
                c_ = models[j]->values[1]- (m_ * models[j]->values[0]);

                double m, c;
                m = (sint +  m_ * cost)/(cost - m_ * sint);
                c = -m * x + y + c_/(cost - m_ * sint);

                double u, v; // w.r.t global
                u = - (m * c)/(m * m + 1);
                v = c/(m * m + 1);

                Eigen::Vector2d measurement = poses[i].inverse() * Eigen::Vector2d (u,v);
                res.push_back (measurement);

            }
            results.push_back(res);
            res.clear();
        }
    }
}


