#include "myslam_mit/slam.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace MYSLAM {

    bool Slam::_init = true;
    int Slam::_frameNum = 0;

    int static keyframe = 0;

	Slam::Slam(ros::NodeHandle& nh)
	       : _nh(&nh), _nextOpt (0) {

		       _graph = new MYSLAM::Graph3();
		       _opt = new MYSLAM::Optimizer3(*_graph);
               _listener = new tf::TransformListener;
	}

	void Slam::callback (const sensor_msgs::PointCloud2ConstPtr& cloud, 
		const sensor_msgs::ImageConstPtr& rgb,
		const sensor_msgs::ImageConstPtr& depth,
		const nav_msgs::OdometryConstPtr& odom,
		const darknet_ros_msgs::BoundingBoxesConstPtr& bb) {

        if (Slam::_frameNum % 15 != 0 || bb->bounding_boxes.size() < 3)  {
            Slam::_frameNum++;
            return;
        }

        Frame f;
        f.odom = *odom;
        f.cloud = *cloud;

        for (int i = 0; i < bb->bounding_boxes.size(); i++) {
            if (bb->bounding_boxes[i].Class == "person") continue;
            f.bb.push_back (bb->bounding_boxes[i]);
        }

        {
            std::unique_lock<std::mutex> lock (_FrameMutex);
            _FrameQueue.push (f);
        }

        Slam::_frameNum++;
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
        int counter = 0;
        for (int i = 0; i < size; i++) {
            if (!pcl::isFinite(inCloud->at(i))) continue;

            counter++;
            
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

        pos[0] = (double) xbar/counter;
        pos[1] = (double) ybar/counter;
        pos[2] = (double) zbar/counter;

        boundary[0] = xmin;
        boundary[1] = ymin;
        boundary[2] = zmin;
    }

    g2o::Vector3 Slam::getObjectPosition (  pcl::PointCloud<pcl::PointXYZ> cloud, 
                                    darknet_ros_msgs::BoundingBox bb) {

        pcl::PointCloud<pcl::PointXYZ> _cloud;

        for (int col = bb.xmin; col < bb.xmax; col++) {
            for (int row = bb.ymin; row < bb.ymax; row++) {
                _cloud.push_back (cloud.at(col,row));
            }
        }

        int size = _cloud.size();
        int counter = 0;
        double xbar = 0.0, ybar = 0.0, zbar = 0.0;
        for (int i = 0; i < size; i++) {
            if (!pcl::isFinite(_cloud.at(i))) continue;

            counter++;
            
            double x = _cloud.at(i).x;
            double y = _cloud.at(i).y;
            double z = _cloud.at(i).z;

            xbar += x;
            ybar += y;
            zbar += z;
        }

        xbar = xbar/(double)counter;
        ybar = ybar/(double)counter;
        zbar = zbar/(double)counter;

        return g2o::Vector3 (xbar, ybar, zbar);
    }

    void Slam::saveData(const sensor_msgs::PointCloud2ConstPtr& cloud,
            const nav_msgs::OdometryConstPtr& odom,
            const darknet_ros_msgs::BoundingBoxesConstPtr& bb) {

        std::ofstream f;
        f.open ("/home/ism/code/rosws/src/pclsave/data3/odom.txt", std::ios::out | std::ios::app);
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

        std::string s = std::to_string(keyframe);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud,*pcl);
        pcl::io::savePCDFileASCII ("/home/ism/code/rosws/src/pclsave/data3/cloud/" + s + ".pcd", *pcl);
        keyframe++;

        f.close();

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

        init();

        for (int i = 1; i < _frame.size(); i++) {

        }
    }

    int Slam::frameToGraph(int id) {

        // number of objects detected in current frame
        int n = _frame[id].n_object_detected;

        // set pose
        Pose3::Ptr pose (new Pose3); _frameToGraphMap[id] = pose->_id;
        Eigen::Vector3d trans (_frame[id].x, _frame[id].y, 0.0);
        Eigen::Matrix3d rot; rot = Eigen::AngleAxisd (_frame[id].t, Eigen::Vector3d::UnitZ());
        pose->_pose.translation() = trans;
        pose->_pose.linear() = rot;

//        set pose-pose measurement
        if (id > 0) {
            int prev_node_id = _frameToGraphMap[id-1];
            int curr_node_id = pose->_id;

            g2o::Isometry3 m = calculateOdometryIncrement (id-1, id);
            _graph->insertPosePoseEdge (prev_node_id, curr_node_id, m);
        }

        // set objects
        for (int i = 0; i < n; i++) {

            ObjectXYZ::Ptr object (new ObjectXYZ);
//            object->_classid = ;
            object->_type = _frame[id].category[i];
            object->_seenBy.insert (pose->_id);
            object->_conf = _frame[id].confidence[i];
            object->_pose = pose->_pose * object->_pose; // sensor model
            _graph->insertNode (object);

            pose->_detectedObjects.insert (object->_id);

            Eigen::Vector3d m (_frame[id].pos[i][0], _frame[id].pos[i][1], _frame[id].pos[i][2]);
            _graph->insertPoseObjectEdge (pose->_id, object->_id, m);
        }

        _graph->insertNode (pose);
    }

    g2o::Isometry3 Slam::calculateOdometryIncrement (int from, int to) {

        SE2 u (_frame[from].x, _frame[from].y, _frame[from].t);
        SE2 v (_frame[to].x, _frame[to].y, _frame[to].t);
        Eigen::Vector3d increment = (u.inverse() * v).toVector();

        Eigen::Vector3d translation (increment[0], increment[1], 0.0);
        Eigen::Matrix3d rotation; rotation = Eigen::AngleAxisd (increment[2], Eigen::Vector3d::UnitZ());

        g2o::Isometry3 result;
        result.translation() = translation;
        result.linear() = rotation;

        return result;
    }

    void Slam::test() {

        g2o::Isometry3 iso1;
        Eigen::Vector3d v1 (1.0, 0.0, 0.0);
        Eigen::Matrix3d r1; r1 = Eigen::AngleAxisd (.0, Eigen::Vector3d::UnitZ());
        iso1.translation() = v1;
        iso1.linear() = r1;
        std::cout << iso1.matrix() << std::endl << std::endl;

        g2o::Isometry3 iso2;
        Eigen::Vector3d v2 (1.0, 0.0, 0.0);
        Eigen::Matrix3d r2; r2 = Eigen::AngleAxisd (M_PI/2, Eigen::Vector3d::UnitZ());
        iso2.translation() = v2;
        iso2.linear() = r2;
        std::cout << iso2.matrix() << std::endl << std::endl;

        std::cout << (iso1.inverse() * iso2).matrix() << std::endl;


    }

    void Slam::thread1() {

        while(1) {
            if (_FrameQueue.size() > 0) {

                Frame f;

                {
                    std::unique_lock<std::mutex> lock (_FrameMutex);
                    f = _FrameQueue.front();
                    _FrameQueue.pop();
                }

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg (f.cloud, *cloud); 

                tf::StampedTransform transform;
                try {
                    _listener->lookupTransform ("/base_link", "/openni_rgb_optical_frame",
                            ros::Time(0), transform);
                } catch (tf::TransformException ex) {
                    ROS_ERROR ("%s", ex.what());
                    ros::Duration (1.0).sleep();
                }

                g2o::Isometry3 currOdom = g2o::Isometry3(odomToSE3Quat(f.odom));

                std::vector<g2o::Vector3> observations;
                std::vector<int> classes;
                for (int i = 0; i < f.bb.size(); i++) {
                    g2o::Vector3 pos = getObjectPosition (*cloud, f.bb[i]);
                    tf::Vector3 v (pos[0], pos[1], pos[2]);
                    v = transform * v;
                    pos << v[0], v[1], v[2];
                    int cls = getObjectClass (f.bb[i].Class);
                    observations.push_back (pos);
                    classes.push_back (cls);

//                    std::cout << f.bb[i].Class << "\t" << pos.transpose() << "\t-->\t" << (currOdom * pos).transpose() << std::endl;
                }
//                std::cout << "---------------------------\n";

                std::vector<int> associations;
                DataAssociation3 da (*_graph);
                da.associateByGrid (currOdom, classes, observations, associations); 
                
//                for (int i = 0; i < associations.size(); i++)
//                    std::cout << associations[i] << "/";
//                std::cout << std::endl;

                Pose3::Ptr pose (new Pose3);
                int currPoseId = pose->_id;
                pose->_pose = currOdom;

                if (!Slam::_init) {
                    _graph->insertPosePoseEdge (_lastPoseId, currPoseId, _lastOdom.inverse() * currOdom);
                }

                for (int i = 0; i < observations.size(); i++) {

                    ObjectXYZ::Ptr o;

                    if (associations[i] == -1) {
                        ObjectXYZ::Ptr ob (new ObjectXYZ);
                        o = ob;
                        o->_classid = getObjectClass (f.bb[i].Class);
                        o->_pose = currOdom * observations[i];
                        o->_type = f.bb[i].Class;
                        o->_conf = f.bb[i].probability;
                        o->_seenBy.insert (currPoseId);
                        _graph->insertNode (o);
                    } else {
                        std::unique_lock<std::mutex> lock (_graph->_nodeMutex);
                        o = _graph->_objectMap[associations[i]];
                        o->_seenBy.insert(currPoseId);
                    }

                    pose->_detectedObjects.insert (o->_id);
                    _graph->insertPoseObjectEdge (currPoseId, o->_id, observations[i]);
                }

                _graph->insertNode (pose);

                _lastPoseId = currPoseId;
                _lastOdom = currOdom;

                if (Slam::_init == true) Slam::_init = false;
            }
        }
    }

    void Slam::thread2() {
        while (!_graph->isReady()) { 
            std::this_thread::sleep_for (std::chrono::microseconds(100));
        }

        std::vector<int> p, o, pp, po, xp, path;

        while(1) {
            path = _graph->getPath();

            if (path.size() > _nextOpt + 2) {
                p.push_back (path[_nextOpt]);
                p.push_back (path[++_nextOpt]);
                p.push_back (path[++_nextOpt]);
            } else
                continue;

            std::set<int> objects;
            for (int i = 0; i < p.size(); i++) {
                std::set<int> obj_i = _graph->getDetectedObjectsFromPose (p[i]);
                objects.insert (obj_i.begin(), obj_i.end());
            }
            std::copy (objects.begin(), objects.end(), std::back_inserter(o));

            std::set<int> passive_poses;
            for (int i = 0; i < o.size(); i++) {
                std::set<int> poses_i = _graph->getPosesFromObjects (o[i]);
                for (int j = 0; j < p.size(); j++) {
                    auto it = poses_i.find (p[j]);
                    if (it != poses_i.end())
                        poses_i.erase (*it);
                }
                passive_poses.insert (poses_i.begin(), poses_i.end());
            }
            std::copy (passive_poses.begin(), passive_poses.end(), std::back_inserter(xp));

            auto ppMap = _graph->getPosePoseMap();
            for (int i = 0; i < p.size() - 1; i++) {
                for (auto it = ppMap.begin(); it != ppMap.end(); it++) {
                    if (it->second->_from == p[i] && it->second->_to == p[i+1])
                        pp.push_back (it->first);
                }
            } 

            auto opMap = _graph->getPoseObjectMap();
            for (int i = 0; i < p.size() - 1; i++) {
                for (int j = 0; j < o.size(); j++) {
                    for (auto it = opMap.begin(); it != opMap.end(); it++) {
                        if (it->second->_from == p[i] && it->second->_to == o[j])
                            po.push_back (it->first);
                    }
                }
            } 

            _opt->localOptimize (p,o,pp,po,xp);

            p.clear();
            o.clear();
            pp.clear();
            po.clear();
            xp.clear();
        }
    }

    g2o::SE3Quat Slam::odomToSE3Quat (nav_msgs::Odometry o) {

        g2o::Vector3 translation (  o.pose.pose.position.x,
                                    o.pose.pose.position.y,
                                    o.pose.pose.position.z);
        g2o::Quaternion rotation (  o.pose.pose.orientation.x,
                                    o.pose.pose.orientation.y,
                                    o.pose.pose.orientation.z,
                                    o.pose.pose.orientation.w);
        g2o::SE3Quat result;
        result.setTranslation (translation);
        result.setRotation (rotation);
        return result;
    }

    int Slam::getObjectClass (std::string c) {

        for (auto it = objectClass.begin(); it != objectClass.end(); it++) {
            if (it->second == c)
                return it->first;
        }

        int newId;
        objectClass.empty() ? newId = 0 : newId = objectClass.rbegin()->first + 1;
        objectClass[newId] = c;
        return newId;
    }
}
