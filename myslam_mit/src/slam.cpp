#include "myslam_mit/slam.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace MYSLAM {

	Slam::Slam(ros::NodeHandle& nh)
	       : _nh(&nh) {

		       _graph = new MYSLAM::Graph();
		       _opt = new MYSLAM::Optimizer(*_graph);
	}

	void Slam::callback (const sensor_msgs::PointCloud2ConstPtr& cloud, 
		const sensor_msgs::ImageConstPtr& rgb,
		const sensor_msgs::ImageConstPtr& depth,
		const nav_msgs::OdometryConstPtr& odom,
		const darknet_ros_msgs::BoundingBoxesConstPtr& bb) {

        std::ofstream f;
        f.open ("/home/ism/Downloads/data.txt", std::ios::out | std::ios::app);
        f   << "odom " 
            << odom->pose.pose.position.x << " "
            << odom->pose.pose.position.y << " "
            << odom->pose.pose.position.z << " "
            << odom->pose.pose.orientation.x << " "
            << odom->pose.pose.orientation.y << " "
            << odom->pose.pose.orientation.z << " "
            << odom->pose.pose.orientation.w << std::endl;

		for (auto it = bb->bounding_boxes.begin(); it != bb->bounding_boxes.end(); it++) {
			f << it->Class << " "
			  << it->probability << " "
			  << it->xmin << " "
			  << it->ymin << " "
			  << it->xmax << " "
			  << it->ymax << std::endl;
		}
        f.close();
	}

	void Slam::loadCloudData() {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		std::string base = "/home/ism/code/rosws/src/pclsave/data2/cloud/";
		std::string ext = ".pcd";

		pcl::visualization::CloudViewer viewer ("Simple");
		for (int i = 0; i < 1275; i++) {
			std::string f = base + std::to_string(i) + ext;
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (f, *cloud) == -1)
			{
				PCL_ERROR ("Gagal");
				return;
			}

			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.translation() << 0.0, 0.0, 0.0;
			transform.rotate (Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud (*cloud, *cloud, transform);

			if (_frame[i].xmin.size() > 0) {
				pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud (new pcl::PointCloud<pcl::PointXYZ>);
				extractCloud (cloud, objCloud, 
						_frame[i].xmin[0],
						_frame[i].xmax[0],
						_frame[i].ymin[0],
						_frame[i].ymax[0]);

				viewer.showCloud (objCloud);
				if (viewer.wasStopped()) break;
			}
		}
	}

	void Slam::loadColoredCloudData() {

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		std::string base = "/home/ism/code/rosws/src/pclsave/data2/cloudcolored/";
		std::string ext = ".pcd";

        std::string title;
		pcl::visualization::CloudViewer viewer (title);
        for (int i = 0; i < 1275; i++) {
            title = std::to_string(i);
            std::string f = base + std::to_string(i) + ext;
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (f, *cloud) == -1)
            {
                PCL_ERROR ("Gagal");
                return;
            }

            int n = _frame[i].n_object_detected;
            if (n > 0) {
                for (int j = 0; j < n; j++) {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nowallCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                    removeStructure(cloud, nowallCloud);
                    extractColoredCloud (nowallCloud, objCloud, 
                            _frame[i].xmin[j],
                            _frame[i].xmax[j],
                            _frame[i].ymin[j],
                            _frame[i].ymax[j]);
                    Eigen::Vector3d pos, boundary;
                    getObjectPosition(objCloud, pos, boundary);
                    _frame[i].pos.push_back(pos);
                    _frame[i].boundary.push_back(boundary);
                }
            }

            viewer.showCloud (cloud);
            if (viewer.wasStopped()) break;
        }
	}

	void Slam::loadRGBData() {

		std::string base = "/home/ism/code/rosws/src/pclsave/data2/rgb-objects/";
		std::string ext = ".jpg.png";

		cv::Mat image;
		cv::namedWindow("x",cv::WINDOW_AUTOSIZE);
		for (int i = 0; i < 1275; i++) {

			image = cv::imread(base+std::to_string(i)+ext,cv::IMREAD_COLOR);
			cv::imshow("x",image);
			cv::waitKey(100);
		}
	}

	void Slam::loadOdomAndObjectData() {

		/* Get odometri data*/
		int index = 0;
		std::ifstream odofile ("/home/ism/code/rosws/src/pclsave/data2/odometri.txt");
		std::string odoline;
		if (odofile.is_open()) {
			while (std::getline (odofile, odoline)) {
				std::stringstream ss (odoline);
				frame f;
				f.index = index;
                f.n_object_detected = 0;
				ss >> f.x >> f.y >> f.t;
				_frame.push_back(f);
				index++;
			}

			odofile.close();
		}

		/* Get sensor data*/
		std::ifstream objfile ("/home/ism/code/rosws/src/pclsave/data2/detected_objects_filtered.txt");
		std::string objline;

		int xoffset = 16;
		int yoffset = 64;
        int xmin = 0;
        int ymin = 0;
        int xmax = 640;
        int ymax = 480;

		index = -1;
		if (objfile.is_open()) {

			while (std::getline (objfile, objline)) {
				if(objline.at(0) == '/') {
					index++;
					continue;
				}

				int coord[4];
				std::string cat;
				int confidence;

				std::stringstream ssobj (objline);
				std::string token;

				int token_in_line = 0;
				while (ssobj >> token) {

					if (token_in_line < 4) coord[token_in_line] = std::stoi (token);
					else if (token_in_line == 4) cat = token;
					else if (token_in_line == 5) confidence = std::stoi(token.substr(0,2)); 

					token_in_line++; 
				}

                int left, right, top, bottom;
                left = coord[0] + xoffset;
                right = coord[1] + xoffset;
                top = coord[2] - yoffset;
                bottom = coord[3] - yoffset;

                left < 0 ? left = 0 : left = left;
                right > 640 ? right = 640 : right = right;
                top < 0 ? top = 0 : top = top;
                bottom > 480 ? bottom = 480 : bottom = bottom;

                _frame[index].n_object_detected++;
				_frame[index].xmin.push_back (left);
				_frame[index].xmax.push_back (right);
				_frame[index].ymin.push_back (top);
				_frame[index].ymax.push_back (bottom);
				_frame[index].category.push_back (cat);
				_frame[index].confidence.push_back (confidence);

//                std::cout << index << " " << left << " " << right << " " << top << " " << bottom << "\n";
			}

			objfile.close();
		}
	}

	void Slam::extractCloud (	const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
					pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud,
					int xmin, int xmax, int ymin, int ymax) {

		for (int col = xmin; col < xmax; col++) {
			for (int row = ymin; row < ymax; row++) {
				outCloud->push_back (inCloud->at(col,row));
			}
		}
	}

	void Slam::extractColoredCloud (	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outCloud,
					int xmin, int xmax, int ymin, int ymax) {

		for (int col = xmin; col < xmax; col++) {
			for (int row = ymin; row < ymax; row++) {
				outCloud->push_back (inCloud->at(row*640+col));
			}
		}
	}

    void Slam::removeStructure (  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outCloud) {

        int rWall = 154;
        int gWall = 112;
        int bWall = 72;
        int rRoof = 61;
        int gRoof = 118;
        int bRoof = 145;
        int rFloor = 91;
        int gFloor = 133;
        int bFloor = 83;

        outCloud = inCloud->makeShared();

        for (int i = 0; i < outCloud->width; i++) {
            int r = outCloud->at(i).r;
            int g = outCloud->at(i).g;
            int b = outCloud->at(i).b;

            if (    (r == rWall || r == rRoof || r == rFloor) &&
                    (g == gWall || g == gRoof || g == gFloor) &&
                    (b == bWall || b == bRoof || b == bFloor)) {
                outCloud->at(i).r = 0;
                outCloud->at(i).g = 0;
                outCloud->at(i).b = 0;
                outCloud->at(i).x = 0.0;
                outCloud->at(i).y = 0.0;
                outCloud->at(i).z = 0.0;
            } 
        }
    }

    void Slam::getObjectPosition (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inCloud, Eigen::Vector3d& pos, Eigen::Vector3d& boundary) {

        int size = inCloud->size();
        double xbar = 0.0, ybar = 0.0, zbar = 0.0;
        double xmin = 10000.0, ymin = 10000.0, zmin = 10000.0;
        for (int i = 0; i < size; i++) {
            if (!pcl::isFinite(inCloud->at(i))) continue;
            
            double x = inCloud->at(i).x;
            double y = inCloud->at(i).y;
            double z = inCloud->at(i).z;

            xbar += x;
            ybar += y;
            zbar += z;

            x < xmin ? xmin = x : xmin = xmin;
            y < ymin ? ymin = y : ymin = ymin;
            z < zmin ? zmin = z : zmin = zmin;
        }

        pos[0] = (double) xbar/size;
        pos[1] = (double) ybar/size;
        pos[2] = (double) zbar/size;

        boundary[0] = xmin;
        boundary[1] = ymin;
        boundary[2] = zmin;
    }

    void Slam::saveData() {

        int size = _frame.size();

        std::ofstream f;
        f.open ("/home/ism/Downloads/data.txt", std::ios::out | std::ios::app);
        for (int i = 0; i < size; i++) {

            f << "ODOM: " << _frame[i].x << " " << _frame[i].y << " " << _frame[i].t << std::endl;
            for (int j = 0; j < _frame[i].n_object_detected; j++) {
                f   << _frame[i].category[j] << " " 
                    << _frame[i].confidence[j] << " " 
                    << _frame[i].pos[j].transpose() << " " 
                    << _frame[i].boundary[j].transpose() << "\n"; 
            }
        }
    }

    void Slam::loadSlamData() {

        // open file
		std::ifstream f ("/home/ism/code/rosws/src/pclsave/data2/data.txt");
		std::string line;
		if (f.is_open()) {
            int idx = -1; 
            int t = 0;
			while (std::getline (f, line)) {
				std::stringstream ss (line);
                std::string token;
                ss >> token;
                if (token == "ODOM:") {
                    idx++;
                    frame fr; fr.index = idx; fr.n_object_detected = 0;
                    _frame.push_back(fr);
                    ss >> _frame[idx].x >> _frame[idx].y >> _frame[idx].t;
                } else {
                    _frame[idx].category.push_back (token);
                    ss >> token; _frame[idx].confidence.push_back (std::stoi(token));
                    Eigen::Vector3d pos;
                    ss >> pos[0] >> pos[1] >> pos[2];
                    _frame[idx].pos.push_back(pos);
                    _frame[idx].n_object_detected++;
                }
			}

			f.close();
		}
    }

    void Slam::run() {
        loadSlamData();
    }
}
