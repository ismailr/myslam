#include "layout_prediction/frame.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

unsigned long Frame::_frameId = 1;

Frame::Frame (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Pose& pose)
    :_cloud (cloud), _pose (&pose), _useCount (0) {
       _id = Frame::_frameId++; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::getCloud ()
{
    return _cloud;
}

Pose& Frame::getPose ()
{
    return *_pose;
}

Frame::~Frame()
{
}
