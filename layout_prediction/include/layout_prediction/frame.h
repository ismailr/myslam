#ifndef _FRAME_H_
#define _FRAME_H_

#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "layout_prediction/pose.h"

class Frame
{
    public:
        typedef std::shared_ptr<Frame> Ptr;
        typedef std::shared_ptr<const Frame> ConstPtr;

        int _useCount;
        bool _goodFrame;

        Frame (pcl::PointCloud<pcl::PointXYZ>::Ptr&, int);
        pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud () { return _cloud; };
        int getPoseId () { return _poseId; };
        int getId () { return _frameId; };

        ~Frame ();

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
        int _poseId;
        static unsigned long _frameId;
};

#endif
