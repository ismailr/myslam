#include "layout_prediction/particle_filter.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    ParticleFilter::ParticleFilter(System& sys) :
        _system (&sys) {
            for (int i = 0; i < N; i++) 
                _particles.push_back (Particle());
        }

    void ParticleFilter::samplePose (Pose::Ptr& pose) {
        for (int i = 0; i < N; i++) {
            _particles[i].pose = SE2 (
                    pose->_pose[0] + gaussian_generator<double>(0.0, 1e-2),
                    pose->_pose[1] + gaussian_generator<double>(0.0, 1e-2),
                    pose->_pose[2] + gaussian_generator<double>(0.0, 5.0*M_PI/180.0));
        }
    }

    void ParticleFilter::updateWeights (WallDetector* wd, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        std::vector<SE2*> poses;
        for (int i = 0; i < N; i++) 
            poses[i] = &_particles[i].pose;

        std::vector<std::vector<Eigen::Vector2d> > z;
        wd->detectFromMultiplePoses (cloud, poses, z);
        dataAssociation (z);
    }

    void ParticleFilter::dataAssociation (std::vector<std::vector<Eigen::Vector2d> >& z) {

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < z[i].size(); j++) {
                

            }
        }
    }

    void ParticleFilter::resample() {

    }
}
