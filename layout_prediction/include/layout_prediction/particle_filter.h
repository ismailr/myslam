#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "layout_prediction/particle.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/wall_detector.h"

using namespace std;

namespace MYSLAM {
    class WallDetector;
    class ParticleFilter {
        public:
            ParticleFilter (System& sys);
            void samplePose (Pose::Ptr& pose);
            void updateWeights (WallDetector* wd, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
            void dataAssociation (std::vector<std::vector<Eigen::Vector2d> >&);
            void resample();

            int N; // number of particles
            vector<Particle> _particles;

        private:
            System* _system;
    };
}

#endif
