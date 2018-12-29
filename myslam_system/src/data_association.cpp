#include <limits>
#include <fstream>
#include <algorithm>
#include <thread>
#include <math.h>

#include "myslam_system/data_association.h"
#include "myslam_system/helpers.h"

#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"

namespace MYSLAM {

    DataAssociation::DataAssociation(Graph& graph)
        :_graph (&graph), N (0)
    {
	const char *fileconfig = "/home/ism/code/rosws/src/myslam/myslam_system/src/myslam.cfg";
	MYSLAM::loadConfFile (fileconfig);

//	tPPF1 = MYSLAM::TPPF1;
//	tPPF2 = MYSLAM::TPPF2;
//	tPPF3 = MYSLAM::TPPF3;
//	tPPF4 = MYSLAM::TPPF4;
    }

    int DataAssociation::associate (Pose::Ptr pose, int classid, Eigen::Vector3d measurement) {
        
        std::set<int> obj = _graph->_objectClassMap[classid];

        if (obj.empty()) return -1;

        SE2 p; p.fromVector (pose->_pose); // pose
        SE2 m; m.fromVector (measurement); // measurement
        Eigen::Vector3d detected_obj_pose  = (p*m).toVector();

        double nearest_distance = std::numeric_limits<double>::max();
        int nearest_obj = -1;
        for (auto it = obj.begin(); it != obj.end(); it++) {

            double d = calculateDistance (detected_obj_pose, (_graph->_objectMap[*it])->_pose);

            if (d < nearest_distance) {
                nearest_distance = d;
                nearest_obj = (_graph->_objectMap[*it])->_id;
            }
        }

        if (nearest_distance < 0.2)
            return nearest_obj;
        else
            return -1;
    }

    double DataAssociation::calculateDistance (Wall::Ptr w1, Wall::Ptr w2) {
        return (double)(w1->_line.xx - w2->_line.xx).norm();
    }

    double DataAssociation::calculateDistance (Wall::Ptr w, Object::Ptr o) {
        Eigen::Vector2d p (o->_pose[0], o->_pose[1]);
        return (double)(w->_line.xx-p).norm();
    }

    double DataAssociation::calculateDistance (Object::Ptr o1, Object::Ptr o2) {
        Eigen::Vector2d p1 (o1->_pose[0], o1->_pose[1]);
        Eigen::Vector2d p2 (o2->_pose[0], o2->_pose[1]);
        return (double)(p1-p2).norm();
    }

    double DataAssociation::calculateDistance (Eigen::Vector3d o1, Eigen::Vector3d o2) {
        Eigen::Vector2d p1 (o1[0], o1[1]);
        Eigen::Vector2d p2 (o2[0], o2[1]);
        return (double)(p1-p2).norm();
    }

    double DataAssociation::calculateTotalDistance (std::vector<int> sequence) {

        int length = sequence.size();
        double dist = 0.0;

        for (int i = 0; i < length-1; i++) {

            Object::Ptr o1 = _graph->_objectMap[sequence[i]];
            Object::Ptr o2 = _graph->_objectMap[sequence[i+1]];

            dist += calculateDistance (o1, o2);
        }

        return dist;
    }

    double DataAssociation::calculateTotalDistance (std::vector<Eigen::Vector3d> sequence) {

        int length = sequence.size();
        double dist = 0.0;

        for (int i = 0; i < length-1; i++) {

            dist += calculateDistance (sequence[i], sequence[i+1]);
        }

        return dist;
    }

    double DataAssociation::calculateDiff (std::vector<int> s1, std::vector<int> s2) {

        double d1 = calculateTotalDistance (s1);
        double d2 = calculateTotalDistance (s2);
        return std::abs (d1-d2);
    }

    std::vector<int> DataAssociation::rearrangeInput (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data)
    {
        int size = data.size();
        std::vector<int> out (size, -1);

        std::map<int, double> arr;

        for (int i = 0; i < size; i++) {

            Eigen::Vector3d measurement = std::get<1>(data[i]);
            SE2 p; p.fromVector (pose->_pose);
            SE2 m; m.fromVector (measurement);
            Eigen::Vector3d o = (p*m).toVector();

            double shortest = std::numeric_limits<double>::max();

            // this prevent two class with no objects
            // having the same shortest values
            if (_graph->_objectClassMap[std::get<0>(data[i])].empty()) {
                shortest = 1000 + i * 1000; // random number
            } else {
                for (auto   it = _graph->_objectClassMap[std::get<0>(data[i])].begin();
                            it != _graph->_objectClassMap[std::get<0>(data[i])].end();
                            it++) {

                    double d = calculateDistance (o, _graph->_objectMap[*it]->_pose);

                    if (d < shortest) {
                        shortest = d;
                    }
                }
            }

            arr[i] = shortest;
        }

        std::vector<double> sorted;
        for (auto it = arr.begin(); it != arr.end(); it++) {
            sorted.push_back (it->second);
        }

        std::sort (sorted.begin(), sorted.end()); 

        for (int i = 0; i < sorted.size(); i++) {
            for (auto it = arr.begin(); it != arr.end(); it++) {
                if (sorted[i] == it->second)
                    out[i] = it->first;
            }
        } 

        // this prevent first two nodes to have the same classids
        // as it has a potential to make a wrong associations
        if (std::get<0>(data[out[0]]) == std::get<0>(data[out[1]])) {
            int tmp = out[1];
            for (int i = 2; i < out.size(); i++) {
                if (std::get<0>(data[out[i]]) != std::get<0>(data[out[0]])) {
                    out [1] = out [i];
                    out[i] = tmp;
                    break;
                }
            }
        }

        return out;
    }

    std::vector<int> DataAssociation::associate (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data) 
    {
        N = data.size();

        std::vector<int> objectsids (N, -1);

        // handle none and single object detection specially
        if (N == 0) return objectsids;
        else if (N == 1) {
            int result = associate (pose, std::get<0>(data[0]), std::get<1>(data[0]));
            objectsids[0] = result;
            return objectsids;
        }

        std::vector<int> rearranged = rearrangeInput (pose, data);
        std::vector<std::tuple<int, Eigen::Vector3d> > arrData;

        for (int i = 0; i < rearranged.size(); i++) {
            arrData.push_back (data[rearranged[i]]);
        }

        // prepare input and data (map)
        std::vector<Eigen::Vector3d> measurements;
        std::vector<int> classids;
        std::vector<double> dz; // distance between measurements
        for (int i = 0; i < arrData.size(); i++) {
            measurements.push_back (std::get<1>(arrData[i]));
            classids.push_back (std::get<0>(arrData[i]));

            if (i < arrData.size() - 1) {
                double d = calculateDistance (std::get<1>(arrData[i]), std::get<1>(arrData[i+1]));
                dz.push_back (d);
            }
        }

        // first two nodes, find exhaustively
        std::set<int> node1 = _graph->_objectClassMap[classids[0]];
        std::set<int> node2 = _graph->_objectClassMap[classids[1]];

        if (node1.empty() && node2.empty()) {
            objectsids[0] = -1; 
            objectsids[1] = -1;
        } else {
            if (node1.empty()) node1.insert (-1); 
            if (node2.empty()) node2.insert (-1); 

            // measurements
            std::vector<Eigen::Vector3d> m; 
            for (int i = 0; i < 2; i++) {
                SE2 p; p.fromVector(pose->_pose);
                SE2 m_; m_.fromVector(measurements[i]);
                m.push_back((p*m_).toVector());
            }

            double shortest = std::numeric_limits<double>::max();
            double shortest_ = std::numeric_limits<double>::max(); // with one or both node -1
            int n1, n2; // current best pair
            int n1_, n2_; // current best pair with one or both node -1

            for (auto it = node1.begin(); it != node1.end(); it++) {
                Eigen::Vector3d o1;
                if (*it == -1) o1 = m[0];
                else o1 = _graph->_objectMap[*it]->_pose;

                for (auto jt = node2.begin(); jt != node2.end(); jt++) {
                    Eigen::Vector3d o2;
                    if (*jt == -1) o2 = m[1];
                    else o2 = _graph->_objectMap[*jt]->_pose;

                    double d = calculateDistance (o1, o2);
                    double diff = std::abs (d - dz[0]);

                    if (*it != -1 || *jt != -1) {
                        if (diff < shortest) {
                            shortest = diff;
                            n1 = *it;
                            n2 = *jt;
                        }
                    } else if (*it != -1 && *jt != -1) {
                        if (diff < shortest_) {
                            shortest_ = diff;
                            n1_ = *it;
                            n2_ = *jt;
                        }
                    }
                }

                if (shortest < 1.0) {
                    objectsids[0] = n1;
                    objectsids[1] = n2;
                } else if (shortest_ < 1.0) {
                    objectsids[0] = n1_;
                    objectsids[1] = n2_;
                } else {
                    objectsids[0] = -1;
                    objectsids[1] = -1;
                } 
            }
        }

        // start next nodes
        for (int i = 2; i < N; i++) {

            std::set<int> nodei = _graph->_objectClassMap[classids[i]];

            // remove object already in the objectsids
            for (auto it = nodei.begin(); it != nodei.end(); it++) {
                auto kt = std::find (objectsids.begin(), objectsids.end(), *it);
                if (kt != objectsids.end())
                    nodei.erase (*it);
            }

            if (nodei.empty()) {
                nodei.insert (-1);
                objectsids[i] = -1;
                continue;
            } 

            // previous node
            Eigen::Vector3d oh;
            if (objectsids[i-1] == -1) {
                SE2 p; p.fromVector(pose->_pose);
                SE2 m; m.fromVector(measurements[i-1]);
                oh = (p*m).toVector();
            } else {
                oh = _graph->_objectMap[objectsids[i-1]]->_pose;
            }

            double shortest = std::numeric_limits<double>::max();
            int ni;

            for (auto it = nodei.begin(); it != nodei.end(); ++it) {

                Eigen::Vector3d oi = _graph->_objectMap[*it]->_pose;

                double d = calculateDistance (oh, oi);
                double diff = std::abs (d - dz[i-1]);

                if (diff < shortest) {
                    shortest = diff;
                    ni = *it;
                } 
            }

            if (shortest < 1.0)
                objectsids[i] = ni;
            else
                objectsids[i] = -1;
        }

        std::vector<int> out (objectsids.size(), -1);
        for (int i = 0; i < objectsids.size(); i++) {
            out[rearranged[i]] = objectsids[i];
        }

        return out;
    }


    std::vector<int> DataAssociation::rearrangeInput2 (Pose::Ptr pose, std::vector<std::tuple<int, Eigen::Vector3d> > data)
    {
        int size = data.size();
        std::vector<int> out (size, -1);

	// first, calculate distances between current pose and all detected objects
	// sort by distance, start from the nearest
	std::map<int, double> distances; // index input --> distance to pose
	for (int i = 0; i < data.size(); i++) {
		Eigen::Vector2d m (std::get<1>(data[i])[0], std::get<1>(data[i])[1]);
		double dist = (double)m.norm();
		distances[i] = dist;
	}

	// sorted distances --> index input
	std::multimap<double,int> sorted_distances = flip_map (distances);

	// copy values of sorted_distances to out
	int index = 0;
	for (std::multimap<double,int>::iterator it = sorted_distances.begin();
			it != sorted_distances.end(); it++) 
		out [index++] = it->second;

	// check whether out[0] and out[1] are the same class
	int class0 = std::get<0>(data[out[0]]);
	int class1 = std::get<0>(data[out[1]]);
	if (class0 == class1) {
		for (int i = 2; i < out.size(); i++) {
			int classi = std::get<0>(data[out[i]]);
			if (classi != class0) {
				int tmp = out[i];
				for (int j = i - 1; j > 1; j--) {
					out[j+1] = out[j];
				}
				out[2] = tmp;
			}
		}
	}

        return out;
    }

    double DataAssociation::calculatePPF1 (Eigen::Vector3d v1, Eigen::Vector3d v2) {
	    return calculateDistance (v1, v2);
    }

    double DataAssociation::calculatePPF2 (Eigen::Vector3d v1, Eigen::Vector3d v2) {
	    SE2 o1; o1.fromVector (v1);
	    SE2 o2; o2.fromVector (v2);
	    SE2 o21 = o1.inverse() * o2;
	    double angle = normalize_angle (atan2 (o21.translation().y(), o21.translation().x()));
	    return angle;
    }

    double DataAssociation::calculatePPF4 (Eigen::Vector3d v1, Eigen::Vector3d v2) {
	    double angle1 = normalize_angle (v1[2]);
	    double angle2 = normalize_angle (v2[2]);
	    return smallest_diff_angle (angle1, angle2);
    }

	std::vector<int> DataAssociation::associateByPPF (Pose::Ptr pose, std::vector<std::tuple<int /*classid*/, Eigen::Vector3d /*measurement*/> > data) {

		N = data.size();
		std::vector<int> out;

		if (N == 0) return out;

		for (int i = 0; i < N; i++) {
			_classids.push_back (std::get<0>(data[i]));
			_measurements.push_back (std::get<1>(data[i]));
		}

		// calculate ppf of input
//		std::ofstream dafile;
//		dafile.open ("/home/ism/code/rosws/result/da-details.log", std::ios::out | std::ios::app);
		for (int i = 0; i < N-1; i++) {
			double ppf1 = calculatePPF1 (_measurements[i],_measurements[i+1]);
			iPPF1.push_back (ppf1); 
			double ppf2 = calculatePPF2 (_measurements[i],_measurements[i+1]);
			iPPF2.push_back (ppf2); 
			double ppf3 = calculatePPF2 (_measurements[i+1],_measurements[i]);
			iPPF3.push_back (ppf3); 
			double ppf4 = calculatePPF4 (_measurements[i],_measurements[i+1]);
			iPPF4.push_back (ppf4); 
//			dafile << i << " -- " << i+1 << " " << ppf1 << " " << ppf2 << " " << ppf3 << " " << ppf4 << std::endl;
		}
//		dafile << "*********************************************" << std::endl;
//		dafile.close();

//		std::ofstream f;
//		f.open ("/home/ism/code/rosws/result/threshold.log", std::ios::out | std::ios::app);
		std::vector<std::vector<int> > out_tmp; 
		associateByPPF1(pose, out_tmp);
//		f << out_tmp.size() << " --> ";
//		associateByPPFN(pose, 2, out_tmp);
//		f << out_tmp.size() << " --> ";
//		associateByPPFN(pose, 3, out_tmp);
//		f << out_tmp.size() << std::endl;

//		for (int i = 0; i < out_tmp.size(); i++) {
//			for (int j = 0; j < out_tmp[i].size(); j++) {
//				f << out_tmp[i][j] << " ";
//			}
//			f << std::endl;
//		}

		out = findBestCandidate (pose, out_tmp);
//		f << "BEST CANDIDATE: ";
//		for (int i = 0; i < out.size(); i++) f << out[i] << " ";
//		f << "----------------------------------------" << std::endl;
//		f.close();

		return out;
	}

	void DataAssociation::associateByPPF1 (Pose::Ptr pose, std::vector<std::vector<int> >& out) {

		for (int i = 0; i < N-1; i++) {
			calculateDiffPPF1 (pose, i, out);
		}
	}

	void DataAssociation::associateByPPFN (Pose::Ptr pose, int ppf, std::vector<std::vector<int> >& out) {

		std::vector<std::vector<int> > current_seq = out;
		out.clear();

		double tPPF;
		switch (ppf) {
			case 2: tPPF = tPPF2; break;
			case 3: tPPF = tPPF3; break;
			case 4: tPPF = tPPF4; break;
			default: tPPF = tPPF2;

		}

		for (int i = 0; i < current_seq.size(); i++) {
			double d = calculateDiffPPFN (pose, ppf, current_seq[i]);
			double d3 = calculateDiffPPFN (pose, 3, current_seq[i]);
			double d4 = calculateDiffPPFN (pose, 4, current_seq[i]);
			if (d < tPPF & d3 < tPPF3 && d4 < tPPF4) out.push_back (current_seq[i]);
		}
	}

	void DataAssociation::calculateDiffPPF1 (Pose::Ptr pose, int idx, std::vector<std::vector<int> >& out) {

		std::vector<std::vector<int> > current_seq = out;
		out.clear();

		// get all objects belong to _classids[idx] if it is the first run
		// otherwise choose from 'out' 
		if (idx == 0) {
			std::set<int> s = _graph->_objectClassMap[_classids[0]];
			s.insert (-1); // new object
			for (auto it = s.begin(); it != s.end(); it++) {
				std::vector<int> v;
				v.push_back (*it);
				current_seq.push_back (v);
			}
		} 

		std::set<int> next_set;
		next_set = _graph->_objectClassMap[_classids[idx+1]];
		next_set.insert (-1);

//		std::ofstream dafile;
//		dafile.open ("/home/ism/code/rosws/result/da-details.log", std::ios::out | std::ios::app);

		for (auto it = current_seq.begin(); it != current_seq.end(); it++) {
			int o1 = it->back();
			for (auto jt = next_set.begin(); jt != next_set.end(); jt++) {
				int o2 = *jt;

				Eigen::Vector3d v1 = (o1 == -1 ? localToGlobal(pose,_measurements[idx]) : _graph->_objectMap[o1]->_pose);
				Eigen::Vector3d v2 = (o2 == -1 ? localToGlobal(pose,_measurements[idx+1]) : _graph->_objectMap[o2]->_pose);
//				double d1 = std::abs (calculatePPF1 (v1,v2) - iPPF1[idx]); 
//				double d2 = std::abs (calculatePPF2 (v1,v2) - iPPF2[idx]); 
//				double d3 = std::abs (calculatePPF2 (v2,v1) - iPPF3[idx]); 
//				double d4 = std::abs (calculatePPF4 (v1,v2) - iPPF4[idx]); 
				double d1 = std::abs (calculatePPF1 (v1,v2) - iPPF1[idx]); 
				double d2 = smallest_diff_angle (calculatePPF2 (v1,v2), iPPF2[idx]); 
				double d3 = smallest_diff_angle (calculatePPF2 (v2,v1), iPPF3[idx]); 
				double d4 = smallest_diff_angle (calculatePPF4 (v1,v2), iPPF4[idx]); 

//				dafile << o1 	<< " -- " << o2
//				       		<< " " << calculatePPF1(v1,v2) 
//						<< " " << calculatePPF2(v1,v2) 
//						<< " " << calculatePPF2(v2,v1) 
//						<< " " << calculatePPF4(v1,v2) << " ";

				if (d1 < tPPF1 && d2 < tPPF2 && d3 < tPPF3 && d4 < tPPF4) {
//					dafile << "OK: " << std::endl;
					std::vector<int> seq = *it;
					seq.push_back (o2);
					out.push_back (seq);
				} 
//				else  dafile << "NOT OK" << std::endl;
			}
		}
//		dafile << "------------------------------------------" << std::endl;
//		dafile.close();
	}

	double DataAssociation::calculateDiffPPFN (Pose::Ptr pose, int ppf, std::vector<int> id) {

		double out;
		for (int i = 0; i < N-1; i++) {
			Eigen::Vector3d o1 = (id[i] == -1 ? localToGlobal(pose,_measurements[i]) : _graph->_objectMap[id[i]]->_pose);
			Eigen::Vector3d o2 = (id[i+1] == -1 ? localToGlobal(pose,_measurements[i+1]) : _graph->_objectMap[id[i+1]]->_pose);

			switch (ppf) {
				case 1: out += std::abs (calculatePPF1 (o1,o2) - iPPF1[i]); break;
				case 2: out += smallest_diff_angle (calculatePPF2 (o1,o2), iPPF2[i]); break;
				case 3: out += smallest_diff_angle (calculatePPF2 (o2,o1), iPPF3[i]); break;
				case 4: out += smallest_diff_angle (calculatePPF4 (o1,o2), iPPF4[i]); break;
				default: out += std::abs (calculatePPF1 (o1,o2) - iPPF1[i]); 
			}
		}
		return out;
	}


	Eigen::Vector3d DataAssociation::localToGlobal (Pose::Ptr p, Eigen::Vector3d v) {

		SE2 local; local.fromVector (v);
		SE2 pose; pose.fromVector (p->_pose);
		return (pose * local).toVector();
	}
		
	std::vector<int> DataAssociation::findBestCandidate (Pose::Ptr pose, std::vector<std::vector<int> >& data) {

		std::vector<int> out;

		if (data.size() == 1) {
			out = data[0];
			return out;
		}

		// look for vector with all entry unequal to -1
		std::map<int, int> minus1;
		int smallestnumofminus1 = 1000;
		for (int i = 0; i < data.size(); i++)
		{
			int numofminus1 = 0;
			for (int j = 0; j < data[i].size(); j++) {
				if (data[i][j] == -1) numofminus1++;
			}
			minus1[i] = numofminus1;

			if (numofminus1 < smallestnumofminus1) smallestnumofminus1 = numofminus1;
		}

		for (auto it = minus1.begin(); it != minus1.end(); it++) {
			if (it->second > smallestnumofminus1) {
				minus1.erase (it);
			}
		}

		int smallest = 100000000;
		int index = 0;
		for (auto it = minus1.begin(); it != minus1.end(); it++) {
			double d1 = calculateDiffPPFN (pose, 1, data[it->first]);
			double d2 = calculateDiffPPFN (pose, 2, data[it->first]);
			double d3 = calculateDiffPPFN (pose, 3, data[it->first]);
			double d4 = calculateDiffPPFN (pose, 4, data[it->first]);
			double d = d1 + d2 + d3 + d4;
			if (d < smallest) {
				smallest = d;
				index = it->first;
			}
		}

//		ofstream f;
//		f.open("/home/ism/code/rosws/result/threshold.log", std::ios::out | std::ios::app);
//		std::map<double,int> s;
//		for (int i = 0; i < data.size(); i++) {
//			double d1 = calculateDiffPPFN (pose, 1, data[i]);
//			double d2 = calculateDiffPPFN (pose, 2, data[i]);
//			double d3 = calculateDiffPPFN (pose, 3, data[i]);
//			double d4 = calculateDiffPPFN (pose, 4, data[i]);
//			double d = d1 + d2 + d3 + d4;
//			s[d] = i;
//		}
//
//		for (auto it = s.begin(); it != s.end(); it++) {
//			for (int j = 0; j < data[it->second].size(); j++) {
//				f << data[it->second][j] << " ";
//			}
//			f << " ------> " << it->first << std::endl;
//		}
//		f << "--------------------------------------------------" << std::endl;
//		f.close();

		out = data[index];
		return out;
	}

    DataAssociation3::DataAssociation3(Graph3& graph)
       :_graph(&graph) {
        _objectMap = _graph->getObjectMap();
        _objectClassMap = _graph->getObjectClassMap();
    }

    void DataAssociation3::associate (  g2o::Isometry3 odom, 
                                        std::vector<int> classes,
                                        std::vector<g2o::Vector3> observations, 
                                        std::vector<int>& associations) {

        if (observations.size() < 3) return;

        _observations = observations;
        _classes = classes;
        _m = _observations.size();
        _odom = odom;

        std::vector<int> H;
        std::vector<std::tuple<int,int>> pairlist;
        for (int i = 0; i < observations.size() - 1; i++) {
            std::set<int> from;
            std::set<int> to = _objectClassMap[classes[i+1]];

            if (i == 0) {
                from = _objectClassMap[classes[i]];
            } else {
                for (auto it = pairlist.begin(); it != pairlist.end(); it++) {
                    from.insert (std::get<1>(*it));
                }
            }

            pairlist.clear();
            findBestPair (3, observations[i], observations[i+1], odom, from, to, pairlist);

            if (i == 0) {
                H.push_back(std::get<0>(pairlist[0]));
                H.push_back(std::get<1>(pairlist[0]));
            } else {
                for (int j = 0; j < pairlist.size(); j++) {
                    if (std::get<0>(pairlist[j]) == H.back())
                            H.push_back (std::get<1>(pairlist[j]));
                }
            }
        }

        associations = H;
    } 

    void DataAssociation3::findBestPair (int n, g2o::Vector3 v1, g2o::Vector3 v2, g2o::Isometry3 odom, std::set<int> s1, std::set<int> s2, std::vector<std::tuple<int,int>>& best) {

        std::map<int,std::tuple<int,int>> pairs;

        g2o::Vector3 v = v2-v1;
        for (auto it = s1.begin(); it != s1.end(); it++) {
            g2o::Vector3 w1;
            if (*it == -1) 
                w1 = v1;
            else
                w1 = odom.inverse() * _objectMap[*it]->_pose;

            for (auto jt = s2.begin(); jt != s2.end(); jt++) {
                if (*jt == *it) continue;
                g2o::Vector3 w2 = odom.inverse() * _objectMap[*jt]->_pose;
                g2o::Vector3 w = w2 - w1;
                double norm = (w-v).norm();
                if (norm > 0.1) continue;
                int key = norm*100000000;
                pairs[key] = std::make_tuple (*it, *jt);
            }
        }

        if (pairs.empty()) 
            best.push_back (std::make_tuple(-1,-1));
        else {
            for (auto it = pairs.begin(); it != pairs.end(); it++) {
                if (n == 0) break;
                best.push_back (it->second);
                n--;
            }
        }
    }

    void DataAssociation3::findBestPairBB (std::vector<int>& H, int i) {

        if (i > _m) {
            if (pairings(H) > pairings(_best))
                    _best = H;
        } else {
            for (auto   it  = _objectClassMap[_classes[i-1]].begin(); 
                        it != _objectClassMap[_classes[i-1]].end(); it++) 
            {
                for (auto   jt  = _objectClassMap[_classes[i]].begin();
                            jt != _objectClassMap[_classes[i]].end(); jt++) {

                        if (binary(i, i+1, *it, *jt, H)) {

                            if (H.empty()) {
                                H.push_back(*it);
                                H.push_back(*jt);
                            } else {
                                H.push_back(*jt);
                            }

                            findBestPairBB (H, i + 1);
                        }
                }
            }

            if (pairings(H) + _m - i > pairings(_best)) {

                if (H.empty()) {
                    H.push_back(-1);
                    H.push_back(-1);
                } else {
                    H.push_back(-1);
                }

                findBestPairBB (H, i + 1);
            }
        }
    }

    int DataAssociation3::pairings (const std::vector<int>& H) {
        int num = 0;
        for (int i = 0; i < H.size(); i++)
            if (H[i] != -1) num++;
        if (num > 2)
            return num;
        return 0;
    }

    bool DataAssociation3::binary (int i, int j, int k, int l, const std::vector<int>& H) {

        g2o::Vector3 v = _observations[j-1] - _observations[i-1];
        g2o::Vector3 w = _odom.inverse() * (_objectMap[l]->_pose - _objectMap[k]->_pose);
        double diff = (w-v).norm();
        if (diff < 0.1)
            return true;
        else
            return false;
    }

    void DataAssociation3::associateByGrid (    g2o::Isometry3 odom, 
                                                std::vector<int> classes,
                                                std::vector<g2o::Vector3> observations, 
                                                std::vector<int>& associations) {

        std::vector<Eigen::Vector3i> _observationsGrid;
        for (int i = 0; i < observations.size(); i++) {
            g2o::Vector3 o = odom * observations[i];
            int x = (int) std::floor (o[0] * 10);
            int y = (int) std::floor (o[1] * 10);
            int z = (int) std::floor (o[2] * 10);
            _observationsGrid.push_back (Eigen::Vector3i (x,y,z));
            std::cout << x << " " << y << " " << z << std::endl;
        }
        std::cout << " ----- \n";

        auto objectClassMap = _graph->getObjectClassMap();
        auto grid = _graph->getGrid();
        auto gridLookup = _graph->getGridLookup();

        for (auto it = gridLookup.begin(); it != gridLookup.end(); it++)
        {
            std::cout << it->first << " "   << std::get<0>(it->second) << " "
                                            << std::get<1>(it->second) << " "
                                            << std::get<2>(it->second) << std::endl;
        }
        std::cout << " ----- \n";

        std::vector<int> best (observations.size(), -1);

        for (int i = 0; i < _observationsGrid.size(); i++) {
            for (auto j = _objectMap.begin(); j != _objectMap.end(); j++) {
                if (j->second->_classid != classes[i]) continue;
                std::vector<int> H (observations.size(), -1);
                for (int k = 0; k < _observationsGrid.size(); k++) {

                    if (k == i) {
                        H[k] = j->first;
                        continue;
                    }

                    Eigen::Vector3i diff = _observationsGrid[k] - _observationsGrid[i];
                    auto cell = gridLookup[j->first];
                    int x_k = std::get<0>(cell) + diff[0];
                    int y_k = std::get<1>(cell) + diff[1];
                    int z_k = std::get<2>(cell) + diff[2];
                    auto check_cell = std::make_tuple (x_k, y_k, z_k);

                    int candidate = -1;
                    if (grid.find(check_cell) != grid.end()) {
                        std::set<int> candidates = grid[check_cell];
                        for (auto cand_it = candidates.begin(); cand_it != candidates.end(); cand_it++) {
                            if (_objectMap[*cand_it]->_classid == classes[k]) {
                                candidate = *cand_it;
                               break;
                            }
                        }
                    }

                    if (candidate == -1)
                        candidate = _graph->matchSurroundingCell (classes[k], std::make_tuple(x_k, y_k, z_k));

                    H[k] = candidate;
                }

                if (pairings(H) > pairings(best))
                    best = H;
            }
        }

        associations = best;
    }
}
