#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include <map>
#include <g2o/types/slam2d/se2.h>

using namespace Eigen;
using namespace std;
using namespace g2o;

namespace MYSLAM {
    class Particle {
        public:
            Particle();
            void addLandmark (Eigen::Vector2d landmark, Eigen::Matrix2d cov); 
            void updateLandmark (int id, Eigen::Vector2d update) { landmarks[id] = update; };
            void updateCovariance (int id, Eigen::Matrix2d update) { covariances[id] = update; };

            double weight;
            SE2 pose;
            map<int, Eigen::Vector2d> landmarks;
            map<int, Eigen::Matrix2d> covariances;
    };
}

#endif
