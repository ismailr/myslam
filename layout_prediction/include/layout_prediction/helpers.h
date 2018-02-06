#ifndef __LAYOUT_PREDICTION_HELPERS__
#define __LAYOUT_PREDICTION_HELPERS__

#include <random>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "se2.h"

using namespace g2o;
using namespace std;

struct line {
    double m, c, r, theta;
    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped q;
    double fitness;
};

struct line_segment {
	geometry_msgs::Point p;
	geometry_msgs::Point q;
}; 

struct line_segment_stamped {
	geometry_msgs::PointStamped p;
	geometry_msgs::PointStamped q;
};

struct line_eq {
	float slope;
	float intercept;
}; 

double calculate_slope (line);
double calculate_slope (line_segment);
double calculate_slope (line_segment_stamped);
line_eq points_to_line_eq (line_segment);
line_eq points_to_line_eq (line_segment_stamped);

double normalize_angle (double a);

class IdGenerator
{
    public:
        IdGenerator(){};
        long getUniqueId (){ return _id++; };


    private:
        static long _id;
};

class Converter
{
    public:
        Converter(){};
        void odomToSE2 (const nav_msgs::OdometryConstPtr& odom, SE2& t);
        void odomCombinedToSE2 (const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined, SE2& t);
};

class Generator
{
    public:
        static unsigned long int id;
};

template<typename T> T gaussian_generator (T mean, T dev)
{
    random_device rd;
    mt19937 mt(rd());
    normal_distribution<double> dist(mean, dev);
    return dist(mt);
}

template<typename T> T uniform_generator (T min, T max)
{
    random_device rd;
    mt19937 mt(rd());
    uniform_real_distribution<double> dist(min, max);
    return dist(mt);
}

namespace nRandMat{
    Eigen::MatrixXf randn(int m, int n);
    Eigen::MatrixXf rand(int m, int n); 
}

#endif
