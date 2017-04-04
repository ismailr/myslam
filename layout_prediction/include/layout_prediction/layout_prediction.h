#ifndef __LAYOUT_PREDICTION_H__
#define __LAYOUT_PREDICTION_H__

#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include "layout_prediction/helpers.h"

typedef pcl::PointCloud<pcl::PointXYZ> pcloud;
typedef pcl::PointCloud<pcl::Normal> pnormal;

void cloud_cb2 (const sensor_msgs::PointCloud2::Ptr&);
void reduce_pcl (const pcloud::Ptr&, const pcloud::Ptr&);
void filter_pcl (const pcloud::Ptr&, const pcloud::Ptr&);
void line_fitting (const pcloud::Ptr&, line_segment_stamped&);
void correct_pcl (const pcloud::Ptr&, const pcloud::Ptr&, const pcloud::Ptr&);
void get_planes (const pcloud::Ptr&, std::vector<pcl::ModelCoefficients>, std::vector<pcl::PointIndices>);

#endif
