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
            void setInit (Eigen::Vector3d pose);
            void makeObservations (WallDetector* wd, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Matrix2d& R);
            void dataAssociation ();
            void resample();
            void resample2();
            Eigen::Matrix2d obvJacobian(SE2 pose, Eigen::Vector2d landmark);
            SE2 getMeanPose();
            void writeMeanPose();

            void stratified_resample(Eigen::VectorXf w, std::vector<int>& keep, float& Neff);
            void stratified_random (unsigned long N, std::vector<float>& di);
            void cumsum (Eigen::VectorXf& w);
            double unifRand();

            int N; // number of particles
            vector<Particle> _particles;
            vector<vector<Vector2d> > _z; // current measurements

            Matrix2d _R; // measurement noise covariance

        private:
            System* _system;
    };
}
#endif
