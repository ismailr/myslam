#include <iostream>
#include <fstream>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "layout_prediction/tracker.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/settings.h"
#include "layout_prediction/visualizer.h"
#include "layout_prediction/icp.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "/home/ism/work/src/libpointmatcher/pointmatcher/DataPointsFiltersImpl.h"

namespace MYSLAM {
    Tracker::Tracker(System& system)
        : _system (&system), 
          _method (MYSLAM::TRACKER_METHOD),
          _lastPCL (new pcl::PointCloud<pcl::PointXYZ>)
    {
        _buffer = new tf2_ros::Buffer;
        _listener2 = new tf2_ros::TransformListener (*_buffer);
        _lastOdom = new SE2();
//        _lastPCL = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    };

//    SE2* Tracker::odomToSE2 (const nav_msgs::OdometryConstPtr& odom)
//    {
////        geometry_msgs::TransformStamped transformStamped;
////        try {
////            transformStamped = _buffer->lookupTransform (
////                    "base_link",
////                    "base_laser_link",
////                    ros::Time(0));
////
////        } catch (tf2::TransformException &ex) {
////            ROS_WARN ("%s", ex.what());
////            ros::Duration(1.0).sleep();
////        }
////
////        double x = transformStamped.transform.translation.x;
////        double y = transformStamped.transform.translation.y;
////        double z = transformStamped.transform.translation.z;
////        double qx = transformStamped.transform.rotation.x;
////        double qy = transformStamped.transform.rotation.y;
////        double qz = transformStamped.transform.rotation.z;
////        double qw = transformStamped.transform.rotation.w;
////
////        tf2::Quaternion q (qx, qy, qz, qw);
////        double angle = tf2::impl::getYaw(q);
////
////        SE2 transform (x, y, angle);
////        SE2 odomSE2;
////
////        Converter c;
////        c.odomToSE2 (odom, odomSE2);
////        SE2 *laserPose = new SE2;
////        *laserPose = odomSE2 * transform;
////        return laserPose;
//
//        Converter c;
//        SE2 *t = new SE2();
//        c.odomToSE2 (odom, *t);
//        return t;
//    }

    void Tracker::trackPoseByOdometry (double* d, Pose::Ptr& pose)
    {
        // heading change
        double dt = d[2] - _lastOdom->rotation().angle();

        // distance
        double x1 = d[0];
        double y1 = d[1];
        double x0 = _lastOdom->translation().x();
        double y0 = _lastOdom->translation().y();
        double dr = sqrt ((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));

//        // handling backward motion
        double current_heading = normalize_angle (_lastPose->_pose[2] + dt);
        if (current_heading < M_PI/2 || current_heading > 3*M_PI/2)
            x1 > x0 ? dr = dr : dr = -dr;
        else if (current_heading > M_PI/2 && current_heading < 3*M_PI/2)
            x1 > x0 ? dr = -dr : dr = dr;
        else if (current_heading == M_PI/2)
            y1 > y0 ? dr = dr : dr = -dr;
        else if (current_heading == 3*M_PI/2)
            y1 > y0 ? dr = -dr : dr = dr;

        double dx = dr * cos (current_heading);
        double dy = dr * sin (current_heading);
        Eigen::Vector3d incr (dx, dy, dt);

        pose->_pose = _lastPose->_pose + incr;
        _lastPose = pose;
        _lastOdom->fromVector (Eigen::Vector3d (d[0], d[1], d[2]));

        ofstream odomfile;
        odomfile.open ("/home/ism/tmp/odom.dat", std::ios::out | std::ios::app);
        odomfile << d[0] << " " << d[1] << " " << d[2] << std::endl;
        odomfile.close();
    }

    void Tracker::trackPoseByConstantVelocityModel (double *data, Pose::Ptr& pose) 
    {
        double deltaTime = _system->_currentTime - _system->_prevTime;

        double vx = data[0];
        double vy = data[1];
        double w  = data[2];
        double x0 = _lastPose->_pose[0];
        double y0 = _lastPose->_pose[1];
        double t0 = _lastPose->_pose[2];

        // motion model
        double x = x0 + (vx * cos(t0) - vy * sin(t0)) * deltaTime;
        double y = y0 + (vx * sin(t0) + vy * cos(t0)) * deltaTime;
        double t = t0 + w * deltaTime;

        pose->_pose << x, y, t;
        _lastPose = pose;

        ofstream actfile;
        actfile.open ("/home/ism/tmp/action.dat", std::ios::out | std::ios::app);
        actfile << pose->_pose[0] << " " << pose->_pose[1] << " " << pose->_pose[2] << std::endl;
        actfile.close();
    }

    void Tracker::trackPoseByScanMatching (pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl, Pose::Ptr& pose, double *d) 
    {
        // heading change
        double dt = d[2] - _lastOdom->rotation().angle();

        // distance
        double x1 = d[0];
        double y1 = d[1];
        double x0 = _lastOdom->translation().x();
        double y0 = _lastOdom->translation().y();
        double dr = sqrt ((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));

        // handling backward motion
        double current_heading = normalize_angle (_lastPose->_pose[2] + dt);
        if (current_heading < M_PI/2 || current_heading > 3*M_PI/2)
            x1 > x0 ? dr = dr : dr = -dr;
        else if (current_heading > M_PI/2 && current_heading < 3*M_PI/2)
            x1 > x0 ? dr = -dr : dr = dr;
        else if (current_heading == M_PI/2)
            y1 > y0 ? dr = dr : dr = -dr;
        else if (current_heading == 3*M_PI/2)
            y1 > y0 ? dr = -dr : dr = dr;

        double dx = dr * cos (current_heading);
        double dy = dr * sin (current_heading);

//
//        double deltaTime = _system->_currentTime - _system->_prevTime;
//
//        double vx = d[0];
//        double vy = d[1];
//        double w  = d[2];
//        double x0 = _lastPose->_pose[0];
//        double y0 = _lastPose->_pose[1];
//        double t0 = _lastPose->_pose[2];
//
//        // motion model
//        double dx = (vx * cos(t0) - vy * sin(t0)) * deltaTime;
//        double dy = (vx * sin(t0) + vy * cos(t0)) * deltaTime;
//        double dt = w * deltaTime;
//
//        pose->_pose << x0 + dx, y0 + dy, t0 + dt;
//        _lastPose = pose;

        pcl::PointCloud<pcl::PointXYZ>::Ptr _f_lastPCL (new pcl::PointCloud<pcl::PointXYZ>);
        filterCloud (*_lastPCL, *_f_lastPCL, 1000);
        
        Eigen::AngleAxisf init_rotation (dt, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f init_translation (dx, dy, 0);
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

        Eigen::Matrix4f T = icpAlignment (_f_lastPCL, pcl, init_guess);

        SE2 TSE2 (T(0,3), T(1,3), atan2 (T(1,0),T(0,0)));
        SE2 lastPose; lastPose.fromVector (_lastPose->_pose);
        SE2 curPose = lastPose * TSE2;
        pose->_pose = curPose.toVector();
        _lastPose = pose;
    }

    void Tracker::trackPoseByParticleFilter (double *data, ParticleFilter* pf) {
        double deltaTime = _system->_currentTime - _system->_prevTime;

        SE2 offset (0.275, 0.0, 0.252);

        double vx = data[0];
        double vy = data[1];
        double w  = data[2];

        for (int i = 0; i < pf->N; i++) {

            double x = pf->_particles[i].pose.translation().x();
            double y = pf->_particles[i].pose.translation().y();
            double t = pf->_particles[i].pose.rotation().angle();

            // motion model
            x += (vx * cos(t) - vy * sin(t)) * deltaTime;
            y += (vx * sin(t) + vy * cos(t)) * deltaTime;
            t += w * deltaTime;

            double xnoise = gaussian_generator<double>(0.0, _system->_sigX);
            double ynoise = gaussian_generator<double>(0.0, _system->_sigX);
            double tnoise = gaussian_generator<double>(0.0, _system->_sigT);

            pf->_particles[i].pose.setTranslation (Eigen::Vector2d (x + xnoise, y + ynoise));
            pf->_particles[i].pose.setRotation (Eigen::Rotation2Dd (t + tnoise));
        }

        ofstream mfile;
        mfile.open ("/home/ism/tmp/action.dat", std::ios::out | std::ios::app);

        double x0 = _lastPose->_pose[0];
        double y0 = _lastPose->_pose[1];
        double t0 = _lastPose->_pose[2];
        double dx = (vx * cos(t0) - vy * sin(t0)) * deltaTime;
        double dy = (vx * sin(t0) + vy * cos(t0)) * deltaTime;
        double dt = w * deltaTime;
        SE2 pose (x0+dx,y0+dy,t0+dt);
        _lastPose->_pose = pose.toVector();

        mfile << pose.toVector()[0] << " " << pose.toVector()[1] << " " << pose.toVector()[2] << std::endl;
        mfile.close();

    }

    void Tracker::trackPoseByPointMatcher (const sensor_msgs::PointCloud2ConstPtr& cloudmsg, Pose::Ptr& pose, double *d) 
    {
        typedef PointMatcher<float> PM;
        typedef PM::DataPoints DP;

        double deltaTime = _system->_currentTime - _system->_prevTime;

        double vx = d[0];
        double vy = d[1];
        double w  = d[2];
        double x0 = _lastPose->_pose[0];
        double y0 = _lastPose->_pose[1];
        double t0 = _lastPose->_pose[2];

        // motion model
        double dx = (vx * cos(t0) - vy * sin(t0)) * deltaTime;
        double dy = (vx * sin(t0) + vy * cos(t0)) * deltaTime;
        double dt = w * deltaTime;

        unique_ptr<DP> ref (new DP (PointMatcher_ros::rosMsgToPointMatcherCloud<float> (*(cloudmsg.get()))));
        unique_ptr<DP> data (new DP (PointMatcher_ros::rosMsgToPointMatcherCloud<float> (*(_lastDP.get()))));

        PM::ICP icp;
        icp.setDefault();
        PM::TransformationParameters T = icp (*data, *data);

        DP data_out (*data);
        icp.transformations.apply (data_out, T);

        std::cout << T << std::endl;

//        Eigen::Vector3d incr (tx, ty, tt);
//        pose->_pose = _lastPose->_pose + incr;
//        _lastPose = pose;
        _lastDP = cloudmsg;

//        ofstream myfile;
//        myfile.open ("/home/ism/tmp/ndt.txt", std::ios::out | std::ios::app);
//        myfile << "TRAN " << tx << " " << ty << " " << tt << std::endl;
//        myfile << "ODOM " << dx << " " << dy << " " << dt << std::endl;
//        myfile << std::endl;
//        myfile.close();
    }

    void Tracker::run () {
        while (true) {
            std::cout << "track is running" << std::endl;
        }
    }

}
