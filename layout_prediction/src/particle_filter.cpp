#include "layout_prediction/particle_filter.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    ParticleFilter::ParticleFilter(System& sys):_system (&sys), N(50) {
        _R = _system->_R;
    }

    void ParticleFilter::setInit (Eigen::Vector3d data) {
        double uniform = 1/(double)N;
        for (int i = 0; i < N; i++) {
            SE2 pose; pose.fromVector(data);
            Particle p;
            p.pose = pose;
            p.weight = uniform;
            _particles.push_back(p);
        }
        std::cout << "DONE POPULATE PARTICLES" << std::endl;
    }

    void ParticleFilter::samplePose (Pose::Ptr& pose) {
        for (int i = 0; i < N; i++) {
            _particles[i].pose = SE2 (
                    pose->_pose[0] + gaussian_generator<double>(0.0, 1e-2),
                    pose->_pose[1] + gaussian_generator<double>(0.0, 1e-2),
                    pose->_pose[2] + gaussian_generator<double>(0.0, 5.0*M_PI/180.0));
        }
    }

    void ParticleFilter::makeObservations (WallDetector* wd, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Matrix2d& R) {
        std::cout << "READY TO DISTRIBUTE POSE" << std::endl;
        std::vector<SE2> poses;
        std::cout << "NUM OF PARTICLES: " << _particles.size() << std::endl;
      
        for (int i = 0; i < N; i++) {
            std::cout << "PICK PARTICLE NUMBER: " << i << std::endl;
            std::cout << _particles[i].pose.toVector().transpose() << std::endl;
            poses.push_back(_particles[i].pose);
        }

        std::cout << "READY TO DETECT WALLS" << std::endl;
        wd->detectFromMultiplePoses (cloud, poses, _z);
    }

    void ParticleFilter::dataAssociation () {

        if (_z.empty()) return;

        for (int i = 0; i < N; i++) { 
            std::map<int, Eigen::Vector2d>::iterator lm_iter = _particles[i].landmarks.begin();
            std::map<int, Eigen::Matrix2d>::iterator cov_iter = _particles[i].covariances.begin();

            std::cout << "LOOP OVER MEASUREMENTS" << std::endl;
            std::cout << _z.size() << std::endl;

            for (int j = 0; j < _z[i].size(); j++) {

                std::cout << "CREATING CACHE" << std::endl;

                struct Cache {
                    Eigen::Matrix2d cov;
                    Eigen::Matrix2d G;
                    Eigen::Matrix2d Z;
                    Eigen::Vector2d lm;
                    Eigen::Vector2d inov;
                } cache;
                std::cout << "DONE" << std::endl;
                
                double maxProb = 0.0;
                int mostLikelyLandmarkId = -1;

                for (;lm_iter != _particles[i].landmarks.end(); lm_iter++, cov_iter++) { 
                    std::cout << "LOOP OVER LANDMARKS" << std::endl;
                    Eigen::Vector2d z_hat = _particles[i].pose.inverse() * lm_iter->second;
                    Eigen::Matrix2d G = obvJacobian (_particles[i].pose, lm_iter->second); 
                    Eigen::Matrix2d Z = G * cov_iter->second * G.transpose() + _R;
                    Eigen::Vector2d inov = _z[i][j] - z_hat;
                    double denom = 1/sqrt (2*M_PI*Z.determinant());
                    double exp = std::exp (-0.5 * inov.transpose() * Z.inverse() * inov);
                    double p = exp * denom;

                    if (p > maxProb) {
                        maxProb = p;
                        mostLikelyLandmarkId = lm_iter->first;
                        cache.cov = cov_iter->second;
                        cache.G = G;
                        cache.Z = Z;
                        cache.lm = lm_iter->second;
                        cache.inov = inov;
                    }
                }

                if (maxProb >= 0.75) {
                    // update landmarks
                    Eigen::Matrix2d K = cache.cov * cache.G.transpose() * cache.Z.inverse();
                    Eigen::Vector2d update = cache.lm + K * cache.inov;
                    Eigen::Matrix2d cov_update = (Eigen::Matrix2d::Identity() - K * cache.G) * cache.cov;
                    _particles[i].updateLandmark (mostLikelyLandmarkId, update);
                    _particles[i].updateCovariance (mostLikelyLandmarkId, cov_update);

                } else {
                    // new landmarks
                    std::cout << "ADDING NEW LANDMARK" << std::endl;
                    Eigen::Vector2d new_landmark = _particles[i].pose * _z[i][j];
                    std::cout << "DONE" << std::endl;
                    Eigen::Matrix2d new_cov = 99 * Eigen::Matrix2d::Identity();
                    _particles[i].addLandmark (new_landmark, new_cov);
                }
            }
            std::cout << "DONE" << std::endl;
        }
    }

    void ParticleFilter::resample() {

    }

    Eigen::Matrix2d ParticleFilter::obvJacobian(SE2 pose, Eigen::Vector2d landmark) {

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
        Eigen::Matrix2d G;
        G << cos(t), sin(t), -sin(t), cos(t);
        return G;
    }

    SE2 ParticleFilter::getMeanPose() {

        double sum = 0.0;
        for (int i = 0; i < N; i++) {
            sum += _particles[i].weight;
        }

        // normalize weight
        for (int i = 0; i < N; i++) {
            _particles[i].weight /= sum;
        }

        double sumx = 0.0;
        double sumy = 0.0;
        double sumt = 0.0;
        for (int i = 0; i < N; i++) {
            double w = _particles[i].weight;
            sumx += _particles[i].pose.translation().x() * w; 
            sumy += _particles[i].pose.translation().y() * w;
            sumt += _particles[i].pose.rotation().angle() * w;
        }

        return SE2(sumx, sumy, sumt);
    }

    void ParticleFilter::writeMeanPose() {
        SE2 mean = getMeanPose();

        ofstream posef;
        posef.open ("/home/ism/tmp/finalpose.dat", std::ios::out | std::ios::app);
        posef << mean.toVector().transpose() << std::endl;
        posef.close();
    }
}
