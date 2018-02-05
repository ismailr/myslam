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

    void ParticleFilter::updateWeights (WallDetector* wd, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Matrix2d& R) {
        std::vector<SE2*> poses;
        for (int i = 0; i < N; i++) 
            poses[i] = &_particles[i].pose;

        std::vector<std::vector<Eigen::Vector2d> > z;
        wd->detectFromMultiplePoses (cloud, poses, z);
        dataAssociation (z, R);
    }

    void ParticleFilter::dataAssociation (std::vector<std::vector<Eigen::Vector2d> >& z,
            Eigen::Matrix2d& R) {

        if (_particles[0].landmarks.size() == 0) {
            // add landmark
            return;
        }

        for (int i = 0; i < N; i++) { 
            std::map<int, Eigen::Vector2d>::iterator lm_iter = _particles[i].landmarks.begin();
            std::map<int, Eigen::Matrix2d>::iterator cov_iter = _particles[i].covariances.begin();
            for (;lm_iter != _particles[i].landmarks.end(); lm_iter++, cov_iter++) {
                Eigen::Matrix2d G;  
                obvJacobian (_particles[i].pose, lm_iter->second, G); 

                double prob = 0.0;
                long id = -1;
                Eigen::Matrix2d Z_tmp;
                Eigen::Vector2d z_tmp, inov_tmp;

                for (int k = 0; k < z[i].size(); k++) {
                    Eigen::Vector2d z_hat = _particles[i].pose.inverse() * lm_iter->second;
                    Eigen::Vector2d inov = z[i][k] - z_hat;
                    Eigen::Matrix2d Z = G * cov_iter->second * G.transpose() + R;

                    double denom = sqrt (2*M_PI*Z.determinant());
                    double exp = std::exp (-0.5 * inov.transpose() * Z.inverse() * inov);
                    double p = exp/denom;

                    if (p > prob) {
                        prob = p;
                        id = lm_iter->first;
                        Eigen::Matrix2d Z_tmp = Z;
                        Eigen::Vector2d inov_tmp = inov;
                        z_tmp = z[i][k];
                    }
                }

                if (prob > 0.75) {
                    // update landmarks
                    Eigen::Matrix2d K = cov_iter->second * G.transpose() * Z_tmp.inverse();
                    Eigen::Vector2d update = lm_iter->second + K * inov_tmp;
                    Eigen::Matrix2d cov_update = (Eigen::Matrix2d::Identity() - K * G) * cov_iter->second;
                    _particles[i].updateLandmark (id, update);
                    _particles[i].updateCovariance (id, cov_update);

                } else {
                    // add landmarks
                    Eigen::Vector2d new_landmark = _particles[i].pose * z_tmp;
                    Eigen::Matrix2d new_cov = (G * R.inverse() * G).inverse();
                    _particles[i].addLandmark (new_landmark, new_cov);
                }
            }
        }
    }

    void ParticleFilter::resample() {

    }

    void ParticleFilter::obvJacobian(SE2 pose, Eigen::Vector2d landmark, Eigen::Matrix2d& G) {

        // sensor model
        // z_hat = h (x, m)
        // z_hat = x.inverse() * m;
        // z_hat_1 = u cos(t) + v sin(t) - x cos(t) - y sin(t)
        // z_hat_2 = -u sin(t) + v cos(t) + x sin(t) - y cos(t)
        // dz_hat_1/du = cos(t)
        // dz_hat_1/dv = sin(t)
        // dz_hat_2/du = -sin(t)
        // dz_hat_2/dv = cos(t)
        double t = pose.rotation().angle();
        G << cos(t), sin(t), -sin(t), cos(t);
    }

    Eigen::Vector2d ParticleFilter::getMeanPose() {
        double cumweight = 0.0;
        Eigen::Vector3d cumPose;
        for (int i = 0; i < N; i++) {
            cumweight += _particles[i].weight;
            cumPose += _particles[i].pose.toVector() * _particles[i].weight;
        }

        return cumPose/cumweight;
    }

    void ParticleFilter::writeMeanPose() {
        Eigen::Vector2d mean = getMeanPose();

        ofstream posef;
        posef.open ("/home/ism/tmp/finalpose.dat", std::ios::out | std::ios::app);
        posef << mean.transpose() << std::endl;
        posef.close();
    }
}
