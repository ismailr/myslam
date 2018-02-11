#include <iostream>
#include <math.h>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/impl/icp_nl.hpp>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/filter.h>

#include "layout_prediction/icp.h"

using namespace std;

void filterCloud(   const pcl::PointCloud<pcl::PointXYZ>& cloud_in, 
                    pcl::PointCloud<pcl::PointXYZ>& cloud_out, 
                    int desired_size){
  cloud_out.clear();

  std::vector<int> non_nan_indices;
  non_nan_indices.reserve(cloud_in.size());
  for (unsigned int i=0; i<cloud_in.size(); i++ ){
    if (!isnan(cloud_in.points.at(i).z)) 
      non_nan_indices.push_back(i);
  }

  cloud_out.reserve(desired_size+1); //just against off-by-one error
  float step = non_nan_indices.size()/static_cast<float>(desired_size);
  step =  step < 1.0 ? 1.0 : step; //only skip, don't use points more than once
  for (float i=0; i<non_nan_indices.size(); i+=step ){
    unsigned int index = non_nan_indices.at(static_cast<unsigned int>(i));
    cloud_out.push_back(cloud_in.points.at(index));
  }
  cloud_out.width=cloud_out.size();
  cloud_out.height=1;
  cloud_out.is_dense=true;
}

Eigen::Matrix4f icpAlignment(   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2, 
                                Eigen::Matrix4f initial_guess){
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>* icp = NULL;
  string icp_method = "icp";
  if(icp_method == "icp"){
    icp = new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
  } else if (icp_method == "icp_nl"){
    icp = new pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>();
  } else{
    icp = new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_out(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices_nonused;
  // Set the input source and target
  icp->setInputSource(cloud_1);
  icp->setInputTarget(cloud_2);
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp->setMaxCorrespondenceDistance(0.05);
  // Set the maximum number of iterations (criterion 1)
  icp->setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  icp->setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp->setEuclideanFitnessEpsilon(1);
  // Perform the alignment
  pcl::PointCloud<pcl::PointXYZ> cloud_registered;//Not used
  icp->align(cloud_registered, initial_guess);
  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  Eigen::Matrix4f transformation = icp->getFinalTransformation();
  std::cout << "has converged:" << icp->hasConverged() << " score: " << icp->getFitnessScore() << std::endl;
  if(icp->hasConverged()){
    delete icp;
    return transformation;
  } else {
    delete icp;
    return initial_guess;
  }
}
