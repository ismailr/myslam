#include "layout_prediction/frame.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

unsigned long Frame::_frameId = 1;

Frame::Frame (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Pose::Ptr& posePtr)
    :_cloud (cloud), _posePtr (posePtr), _useCount (0), _goodFrame (true) {
       _id = Frame::_frameId++; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::getCloud ()
{
    return _cloud;
}

Pose::Ptr Frame::getPose ()
{
    return _posePtr;
}

Frame::~Frame()
{
}
