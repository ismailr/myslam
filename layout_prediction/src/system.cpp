#include <stdlib.h>
#include <string>
#include <fstream>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>
#include <tf2/impl/utils.h>

#include "layout_prediction/system.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/settings.h"
#include "layout_prediction/particle_filter.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/median_filter.h>

static int marker_id = 0;

namespace MYSLAM {
    unsigned int long System::_frameCounter = 1;

    System::System(ros::NodeHandle nh)
        :_rosnodehandle (nh), _init (true), _method (BA) {
        _tracker = new Tracker(*this);
        _wallDetector = new WallDetector (*this);
        _graph = new Graph (*this);
        _optimizer = new Optimizer (*this, *_graph);
        _visualizer = new Visualizer (_rosnodehandle, *this, *_graph);
        _listener = new tf::TransformListener;
        _buffer = new tf2_ros::Buffer;
        _listener2 = new tf2_ros::TransformListener (*_buffer);
        _pf = new ParticleFilter(*this);
        _ekfm = new EKFMapper (*this);

        _Q <<   _sigX*_sigX, 0.0, 0.0,
                0.0, _sigX*_sigX, 0.0,
                0.0, 0.0, _sigT*_sigT;            

        _R <<   _sigW*_sigW, 0.0,
                0.0, _sigW*_sigW;
    };

    void System::readSensorsData (
            const sensor_msgs::PointCloud2ConstPtr& cloudmsg, 
//            const sensor_msgs::ImageConstPtr& rgb,
//            const sensor_msgs::ImageConstPtr& depth,
            const nav_msgs::OdometryConstPtr& odom//,
//            const nav_msgs::OdometryConstPtr& action,
//            const nav_msgs::OdometryConstPtr& wodom,
//            const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomCombined
            )
    {
//        std::ofstream allfile;
//        allfile.open ("/home/ism/tmp/all.dat", std::ios::out | std::ios::app);
//        clock_t start = clock();
        _currentTime = ros::Time::now().toSec();
//        _currentTime = action->header.stamp.toSec();

//        double vx = action->pose.pose.position.x;
//        double vy = action->pose.pose.position.y;
//        double w  = action->pose.pose.orientation.z;
//
//        if (!_init && vx == 0 && vy == 0 && w == 0)
//            return;

        // convert sensor data 
        double datax = 0.0, datay = 0.0, datat = 0.0;

        Converter c;
        SE2 *t = new SE2();

        if (_tracker->_method == _tracker->USE_CONSTANT_VELOCITY_MODEL 
                || _tracker->_method == _tracker->USE_PARTICLE_FILTER 
                || _tracker->_method == _tracker->USE_POINT_MATCHER) {
//            datax = action->pose.pose.position.x;
//            datay = action->pose.pose.position.y;
//            datat = action->pose.pose.orientation.z;
        } else {
            if (_tracker->_method == _tracker->USE_ODOMETRY
                || _tracker->_method == _tracker->USE_SCAN_MATCHING) 
                c.odomToSE2 (odom, *t);
//            else if (_tracker->_method == _tracker->USE_ODOMETRY_IMU)
//                c.odomCombinedToSE2 (odomCombined, *t);
            else if (_tracker->_method == _tracker->USE_GMAPPING) {
//                Eigen::Vector3d o;
//                o <<    wodom->pose.pose.position.x, 
//                        wodom->pose.pose.position.y,
//                        wodom->pose.pose.position.z;
//                t->fromVector(o);
            }


            datax = t->translation().x();
            datay = t->translation().y();
            datat = t->rotation().angle();
        }

        double data[3] = {datax, datay, datat};

        // convert pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloudmsg, *cloud);

//        pcl::PointCloud<pcl::PointXYZ> medfilteredCloud;
//        pcl::MedianFilter<pcl::PointXYZ> medfilter; 
//        medfilter.setInputCloud (cloud);
//        medfilter.applyFilter (*cloud);

        try {
            _listener->waitForTransform (   
                    MYSLAM::PCL_FRAME,
                    "openni_rgb_optical_frame",
                    ros::Time::now(), 
                    ros::Duration(10.0));

            pcl_ros::transformPointCloud (
                    MYSLAM::PCL_FRAME,
                    *cloud, *cloud, *_listener);
        } 
        catch (tf::TransformException &ex) {
            ROS_ERROR ("%s", ex.what());
        }

        Pose::Ptr pose (new Pose);

        if (_init) // set prior
        {
            pose->_pose = t->toVector();
            _tracker->setPrior (pose);
            _tracker->setFirstPCL (cloud); // for scan matching
            _tracker->setFirstDP (cloudmsg); // for point matcher
            _pf->setInit (pose->_pose); // for particle filter
        } else {
            if (_tracker->_method == _tracker->USE_CONSTANT_VELOCITY_MODEL)
                _tracker->trackPoseByConstantVelocityModel (data, pose);
            else if (_tracker->_method == _tracker->USE_ODOMETRY
                    || _tracker->_method == _tracker->USE_ODOMETRY_IMU)
                _tracker->trackPoseByOdometry (data, pose);
            else if (_tracker->_method == _tracker->USE_SCAN_MATCHING)
                _tracker->trackPoseByScanMatching (cloud, pose, data);
            else if (_tracker->_method == _tracker->USE_PARTICLE_FILTER)
                _tracker->trackPoseByParticleFilter (data, _pf);
            else if (_tracker->_method == _tracker->USE_POINT_MATCHER)
                _tracker->trackPoseByPointMatcher (cloudmsg, pose, data);
            else if (_tracker->_method == _tracker->USE_GMAPPING)
                pose->_pose = t->toVector();
        }

        if (_method == BA) {
            _graph->_poseMap[pose->_id] = pose;
            _graph->_activePoses.push_back (pose->_id);
            _graph->_poseList.push_back (pose->_id);

            // detectwall
            std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> > walls;
            _wallDetector->detect (pose, cloud, walls);

          // data association
            for (size_t i = 0; i < walls.size(); i++)
            {
                Wall::Ptr w = std::get<0>(walls[i]);
                w = _graph->dataAssociation (w);
                std::tuple<int, int> m (pose->_id, w->_id);
                _graph->_poseWallMap[m] = std::get<1>(walls[i]);
                _graph->_activeWalls.insert (w->_id);
                _graph->_activeEdges.push_back (m);
                pose->_detectedWalls.insert (w->_id);
                w->_seenBy.insert (pose->_id);
            }

            int N = _graph->_activePoses.size();
            int M = _graph->_activeWalls.size();
            int E = _graph->_activeEdges.size();
            int V = 3*N + 2*M;

//            std::ofstream lfile;
//            lfile.open ("/home/ism/tmp/numoflandmark.dat",std::ios::out | std::ios::app);
//            lfile   << System::_frameCounter << " " 
//                    << N << " " << M << " " << E << " " << V << std::endl;
//            lfile.close();

//            if (E >= V)
            if (System::_frameCounter % 3 == 0)
//            if (M >= 2 && N >= 2)
            {
//                _visualizer->visualizeWallOptimizedPq();
                _optimizer->localOptimize();

//                std::ofstream wallfile;
//                wallfile.open ("/home/ism/tmp/wall.dat", std::ios::out);
//                std::map<int, Wall::Ptr>::iterator it;
//                for (it = _graph->_wallMap.begin(); it != _graph->_wallMap.end(); it++) {
//                    wallfile    << it->second->_line.p.transpose() << " "
//                                << it->second->_line.q.transpose() << std::endl;
//                }
//                wallfile.close();
            }
        } else if (_method == PF) {
            _pf->makeObservations (_wallDetector, cloud, _R);
            _pf->dataAssociation ();
            _pf->writeMeanPose();
            _pf->resample2();
            _pf->_z.clear();
        } else if (_method == EKF) {
            _graph->_poseMap[pose->_id] = pose;

            double x = pose->_pose[0];
            double y = pose->_pose[1];
            double t = pose->_pose[2];

            SE2 poseSE2 (x, y, t);

//            std::ofstream wdfile;
//            wdfile.open ("/home/ism/tmp/t_walldetection.dat", std::ios::out | std::ios::app);
//            wdfile  << System::_frameCounter << " ";

            std::vector<std::tuple<Wall::Ptr, Eigen::Vector2d> > walls;
//            clock_t wdstart = clock();
            _wallDetector->detect2 (pose, cloud, walls);
//            wdfile << (double)(clock() - wdstart)/CLOCKS_PER_SEC << std::endl;
//            wdfile.close();

            // data association
//            std::ofstream dafile;
//            dafile.open ("/home/ism/tmp/t_dataassociation.dat", std::ios::out | std::ios::app);
//            dafile << _graph->_wallMap.size() << " ";
//            clock_t start = clock();
//
            // Jacobian
            // z_hat_xx = xx*cost + xy*sint - x*cost - y*sint;
            // z_hat_xy = -xx*sint + xy*cost + x*sint - y*cost;
            // | d(z_hat_xx)/dxx d(z_hat_xx)/dxy | = |  cost sint |
            // | d(z_hat_xy)/dxx d(z_hat_xy)/dxy |   | -sint cost |
            Eigen::Matrix2d H;
            H << cos(t), sin(t), -sin(t), cos(t);

            for (size_t i = 0; i < walls.size(); i++)
            {
                Eigen::Vector2d z = std::get<1>(walls[i]);
                Wall::Ptr w = std::get<0>(walls[i]);
                bool newlandmark =_graph->dataAssociationEKF (pose->_id, w, z);

                if (!newlandmark) {
                    // update landmarks
                    Eigen::Vector2d z_hat = poseSE2.inverse() * w->_line.xx;
                    Eigen::Vector2d e = z - z_hat;
                    Eigen::Matrix2d S = w->cov;

                    // Kalman Gain
                    // K = S * H^T * (R + H * S * H^T).inverse()
                    Eigen::Matrix2d K;
                    K = S * H.transpose() * (_R + H * S * H.transpose()).inverse();

                    // update landmark mean
                    _graph->_wallMap[w->_id]->_line.xx = w->_line.xx + K * e;

                    // update covariance
                    _graph->_wallMap[w->_id]->cov = (Eigen::Matrix2d::Identity() - K * H) * S;
                    _graph->_wallMap[w->_id]->updateParams();

                    // update segment
                    Eigen::Vector2d p = std::get<0>(walls[i])->_line.p;
                    Eigen::Vector2d q = std::get<0>(walls[i])->_line.q;
                    _graph->_wallMap[w->_id]->updateSegment (p, q);
                } else {
                    // new landmarks
//                    w = std::get<0>(walls[i]);
                    w->cov = (H.transpose() * _R.inverse() * H).inverse();
                    _graph->_wallMap[w->_id] = w;
                }
                std::tuple<int, int> m (pose->_id, w->_id);
                _graph->_poseWallMap[m] = z;
                pose->_detectedWalls.insert (w->_id);
                w->_seenBy.insert (pose->_id);
            }
//            dafile << (double)(clock() - start)/CLOCKS_PER_SEC << std::endl;
//            dafile.close();

            std::ofstream wallfile;
            wallfile.open ("/home/ism/tmp/wall.dat", std::ios::out);
            std::map<int, Wall::Ptr>::iterator it;
            for (it = _graph->_wallMap.begin(); it != _graph->_wallMap.end(); it++) {
                wallfile    << it->second->_line.p.transpose() << " "
                            << it->second->_line.q.transpose() << std::endl;
            }
            wallfile.close();

            std::ofstream posefile;
            posefile.open ("/home/ism/tmp/finalpose.dat", std::ios::out | std::ios::app);
            posefile << pose->_pose.transpose() << std::endl;
            posefile.close();
        }

        _prevTime = _currentTime;
        _init = false;
        System::_frameCounter++;

//        allfile << System::_frameCounter << " ";
//        allfile << (double)(clock() - start)/CLOCKS_PER_SEC << std::endl;
//        allfile.close();

        pcl::PointCloud<pcl::PointXYZ>::Ptr proj (new pcl::PointCloud<pcl::PointXYZ>);
        proj = cloud->makeShared();

        for (int i = 0; i < cloud->size(); i++) {
//            proj->points[i].z = 0.0;
            if (proj->points[i].z <= 1.5 || proj->points[i].z >= 2.0) {
                proj->points[i].x = 0.0;
                proj->points[i].y = 0.0;
                proj->points[i].z = 0.0; }
//            else {
//                proj->points[i].z = 0.0;
//            }
        }

//        _visualizer->visualizeCloud(proj);
//        _visualizer->visualizeWallOptimizedPq ();

    }
}

