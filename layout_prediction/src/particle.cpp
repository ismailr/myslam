#include "layout_prediction/particle.h"
#include "layout_prediction/helpers.h"

namespace MYSLAM {
    Particle::Particle(){}

    void Particle::addLandmark (Eigen::Vector2d landmark, Eigen::Matrix2d cov) {
        unsigned long int id = Generator::id++;
        landmarks[id] = landmark;
        covariances[id] = cov;
    }
}

