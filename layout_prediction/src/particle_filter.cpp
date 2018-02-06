#include "layout_prediction/particle_filter.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    ParticleFilter::ParticleFilter(System& sys):_system (&sys), N(100) {
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
    }

    void ParticleFilter::makeObservations (WallDetector* wd, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Matrix2d& R) {
        std::vector<SE2> poses;
      
        for (int i = 0; i < N; i++) {
            poses.push_back(_particles[i].pose);
        }

        wd->detectFromMultiplePoses (cloud, poses, _z);
    }

    void ParticleFilter::dataAssociation () {

        if (_z.empty()) return;

        for (int i = 0; i < N; i++) { 
            std::map<int, Eigen::Vector2d>::iterator lm_iter = _particles[i].landmarks.begin();
            std::map<int, Eigen::Matrix2d>::iterator cov_iter = _particles[i].covariances.begin();

            double new_weight = 1.0;
            for (int j = 0; j < _z[i].size(); j++) {

                struct Cache {
                    Eigen::Matrix2d cov;
                    Eigen::Matrix2d G;
                    Eigen::Matrix2d Z;
                    Eigen::Vector2d lm;
                    Eigen::Vector2d inov;
                } cache;
                
                double maxProb = 0.0;
                int mostLikelyLandmarkId = -1;

                for (;lm_iter != _particles[i].landmarks.end(); lm_iter++, cov_iter++) { 
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
                    new_weight *= maxProb;

                } else {
                    // new landmarks
                    Eigen::Vector2d new_landmark = _particles[i].pose * _z[i][j];
                    Eigen::Matrix2d new_cov = 99 * Eigen::Matrix2d::Identity();
                    _particles[i].addLandmark (new_landmark, new_cov);
                }
            }
            _particles[i].weight = new_weight;
        }
    }

    void ParticleFilter::resample() {
        std::vector<double> cumdist;
        double cum = 0.0;
        for (int i = 0; i < N; i++) {
            cum += _particles[i].weight;
            cumdist.push_back (cum);
        }

        // copy the particles
        std::vector<Particle> p = _particles;

        // then clear the original
        _particles.clear();

        // fill with resampling result
        double new_weight = 1.0/(double) N;
        for (int i = 0; i < N; i++) {
            double pick = uniform_generator<double>(0,1);

//            auto jt = std::find_if (cumdist.begin(), cumdist.end(), 
//                    [pick](const std::vector<double>& e){ return e >= pick; });

            std::vector<double>::iterator jt;
            jt = std::upper_bound (cumdist.begin(), cumdist.end(), pick);

            if (jt != cumdist.end()) {
                int index = std::distance (cumdist.begin(), jt);
                p[index].weight = new_weight;
                _particles.push_back (p[index]);
            }
        } 
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

    void ParticleFilter::resample2() {

        double Nmin = 0.75 * N;
        Eigen::VectorXf w(N);

        for (int i = 0; i < N; i++) {
            w(i) = _particles[i].weight;
        }

        float ws = w.sum();
        for (int i = 0; i < N; i++) {
            _particles[i].weight /= ws;
        }

        float Neff = 0;
        std::vector<int> keep;

        stratified_resample(w, keep, Neff);

        std::vector<Particle> old_particles = std::vector<Particle>(_particles);
        _particles.resize(keep.size());	

        if (Neff < Nmin) {
            for(int i = 0; i < keep.size(); i++) {
                _particles[i] = old_particles[keep[i]]; 	
            }

            float new_w = 1.0f/(float) N;
            for (int i = 0; i < N; i++) {
                _particles[i].weight = new_w;
            }
        }
    }

    void ParticleFilter::stratified_resample(Eigen::VectorXf w, std::vector<int>& keep, float& Neff) {

        Eigen::VectorXf wsqrd(w.size());
        double w_sum = w.sum();

        for (int i = 0; i < w.size(); i++) {
            // FIXME: matlab version is: w = w / sum(w)
            w(i) = w(i)/w_sum;
            wsqrd(i) = pow(w(i),2);
        }

        Neff = 1/wsqrd.sum();

        int len = w.size();
        keep.resize(len);
        for (int i = 0; i < len; i++) {
            keep[i] = -1;
        }

        std::vector<float> select;
        stratified_random(len, select);
        cumsum(w);    

        int ctr=0;
        for (int i = 0; i < len; i++) {
            while ((ctr < len) && (select[ctr] < w(i))) {
                keep[ctr] = i;
                ctr++;
            }
        }
    }

    void ParticleFilter::cumsum (Eigen::VectorXf& w) 
    {
        Eigen::VectorXf csumVec (w.size());
        for (int i = 0; i < w.size(); i++) {
            float sum = 0;
            for (int j = 0; j <= i; j++) {
                sum += w(j);
            }			
            csumVec(i) = sum;
        }

        w = Eigen::VectorXf(csumVec); //copy constructor. Double check
    }

    void ParticleFilter::stratified_random (unsigned long _N, std::vector<float>& di)
    { 
        float k = 1.0/(float)_N;
       
        //deterministic intervals
        float temp = k/2;
        while (temp < (1-k/2)) {
            di.push_back(temp);
            temp = temp+k;
        }
       
        // FIXME: when set NPARTICLES = 30, this whill failed
        assert(di.size() == _N); 
        
        //dither within interval
        std::vector<float>::iterator diter; 
        for (diter = di.begin(); diter != di.end(); diter++) {
            *diter = (*diter) + unifRand() * k - (k/2);
        }
    }

    double ParticleFilter::unifRand()
    {
        return rand() / double(RAND_MAX);
    }
}

