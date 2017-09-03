#ifndef _FRAME_H_
#define _FRAME_H_

#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "layout_prediction/pose.h"

class Pose;
class Frame
{
    public:
        int _useCount;
        static unsigned long _frameId;
        unsigned long _id;
        bool _goodFrame;

        Frame (pcl::PointCloud<pcl::PointXYZ>::Ptr, Pose&);
        pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud ();
        Pose& getPose ();

        ~Frame ();

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
        Pose *_pose;


};

#endif
