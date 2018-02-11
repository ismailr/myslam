#ifndef _ICP_H_
#define _ICP_H_

#include <pcl/point_types.h>

///Filter NaNs. Uniformly subsample the remaining points to achieve the desired size
void filterCloud(   const pcl::PointCloud<pcl::PointXYZ>& cloud_in, 
                    pcl::PointCloud<pcl::PointXYZ>& cloud_out, 
                    int desired_size);

Eigen::Matrix4f icpAlignment(   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2, 
                                Eigen::Matrix4f initial_guess);
#endif

