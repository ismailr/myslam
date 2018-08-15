#ifndef _DATA_ASSOCIATION_H_
#define _DATA_ASSOCIATION_H_

#include "myslam_system/graph.h"
#include "myslam_system/settings.h"

namespace MYSLAM {
    class DataAssociation {
        public:
            DataAssociation (Graph&);

            // args: robot's pose, vector<classid, measurements>
            // return: set of objects' ids
		std::vector<int> associate (Pose::Ptr, std::vector<std::tuple<int, Eigen::Vector3d> >); 
		std::vector<int> associateByPPF (Pose::Ptr, std::vector<std::tuple<int, Eigen::Vector3d> >); 
		int associate (Pose::Ptr, int, Eigen::Vector3d);

        private:
            Graph* _graph;

		std::vector<int> rearrangeInput (Pose::Ptr, std::vector<std::tuple<int, Eigen::Vector3d> >);
		std::vector<int> rearrangeInput2 (Pose::Ptr, std::vector<std::tuple<int, Eigen::Vector3d> >);
		void associateByPPF1 (Pose::Ptr, std::vector<std::vector<int> >&);
		void associateByPPFN (Pose::Ptr, int, std::vector<std::vector<int> >&);
		double calculateDistance (Wall::Ptr, Wall::Ptr);
		double calculateDistance (Wall::Ptr, Object::Ptr);
		double calculateDistance (Object::Ptr, Object::Ptr);
		double calculateDistance (Eigen::Vector3d, Eigen::Vector3d);
		double calculateTotalDistance (std::vector<int>);
		double calculateTotalDistance (std::vector<Eigen::Vector3d>);
		double calculateDiff (std::vector<int>, std::vector<int>);
		double calculatePPF1 (Eigen::Vector3d v1, Eigen::Vector3d v2);
		double calculatePPF2 (Eigen::Vector3d v1, Eigen::Vector3d v2);
		double calculatePPF4 (Eigen::Vector3d v1, Eigen::Vector3d v2);
		void calculateDiffPPF1 (Pose::Ptr, int /*index of input*/, std::vector<std::vector<int> >& /*out*/);
		double calculateDiffPPFN (Pose::Ptr, int /*ppf*/, std::vector<int>);
		std::vector<int> findBestCandidate (Pose::Ptr, std::vector<std::vector<int> >&);

	    Eigen::Vector3d localToGlobal (Pose::Ptr, Eigen::Vector3d);

	    // threshold
//	    double tPPF1 = 1.0;
//	    double tPPF2 = 5.0 * M_PI/180.0;
//	    double tPPF3 = tPPF2;
//	    double tPPF4 = 1.0 * M_PI/180.0;

	    double tPPF1; 
	    double tPPF2; 
	    double tPPF3; 
	    double tPPF4;

	    std::vector<double> iPPF1; // PPF1 of input
	    std::vector<double> iPPF2;
	    std::vector<double> iPPF3;
	    std::vector<double> iPPF4;

	    int N; // input size

	    std::vector<int> _classids;
	    std::vector<Eigen::Vector3d> _measurements;
    };
}

#endif
