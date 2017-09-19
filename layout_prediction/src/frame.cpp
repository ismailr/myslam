#include "layout_prediction/frame.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

Frame::Frame (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int poseId)
    :_cloud (cloud), _poseId (poseId), _useCount (0), _goodFrame (true), _frameId (poseId) {
}

Frame::~Frame()
{
}
