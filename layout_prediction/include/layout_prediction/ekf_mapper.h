#ifndef _EKF_MAPPER_H_
#define _EKF_MAPPER_H_

#include "layout_prediction/system.h"
#include "layout_prediction/wall.h"

namespace MYSLAM {
    class EKFMapper {
        public:
            EKFMapper(System& sys);

            void addWall (Wall::Ptr&);
            void updateWall (int id);

            // wall container
            std::map<int, Wall::Ptr> wallMap;
            Eigen::Vector2d inverseSensorModel (const Eigen::Vector2d&);

        private:
            System* _sys;

            // initial covariance for any new wall
            const Eigen::Matrix2d cov0 = Eigen::Matrix2d::Identity() * 99;

    };
}

#endif
