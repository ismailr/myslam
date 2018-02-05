#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include <map>
#include <g2o/types/slam2d/se2.h>

#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"

using namespace Eigen;
using namespace std;

namespace MYSLAM {
    class Particle {
        public:
            Particle();
            void addLandmark (Vector2d landmark, Matrix2d cov); 
            void updateLandmark (int id, Vector2d update) { landmarks[id] = update; };
            void updateCovariance (int id, Matrix2d update) { covariances[id] = update; };

            double weight;
            SE2 pose;
            map<int, Vector2d> landmarks;
            map<int, Matrix2d> covariances;
    };
}

#endif
